#!/usr/bin/env bash

# =============================================================
# NFS 服务器自动配置脚本（Ubuntu）
# 功能：安装与配置 NFS、设置自启动、生成导出项、服务验证
# 运行：sudo bash config/server/nfs_config.sh
# 说明：脚本入口为 main()，函数职责单一，中文日志与错误处理完整
# =============================================================

set -euo pipefail
trap 'log_error "执行失败：第$LINENO行，退出码=$?"' ERR
export DEBIAN_FRONTEND=noninteractive

# ----------------------------- 日志与通用工具 -----------------------------
ts() { date '+%Y-%m-%d %H:%M:%S'; }
log_info() { echo "[INFO $(ts)] $*"; }
log_warn() { echo "[WARN $(ts)] $*"; }
log_error() { echo "[ERROR $(ts)] $*" >&2; }

command_exists() { command -v "$1" >/dev/null 2>&1; }

install_packages() {
  # 功能：统一安装多个包并校验安装结果
  # 参数：包名列表
  local pkgs=("$@")
  [ ${#pkgs[@]} -eq 0 ] && return 0
  log_info "安装依赖包：${pkgs[*]}"
  apt-get update -y
  apt-get install -y "${pkgs[@]}"
  for p in "${pkgs[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || { log_error "依赖安装失败：$p"; exit 1; }
  done
}

require_tool_iproute2() {
  # 功能：确保可用 ip 命令
  command_exists ip || install_packages iproute2
}

require_tool_net_tools() {
  # 功能：确保可用 ifconfig 命令（回退用）
  command_exists ifconfig || install_packages net-tools
}

require_tool_nfs_common() {
  # 功能：确保可用 showmount（验证导出）
  command_exists showmount || install_packages nfs-common
}

ensure_dependencies() {
  # 功能：汇总依赖自检
  require_tool_iproute2
  require_tool_net_tools
  require_tool_nfs_common
}

# ----------------------------- 权限检查 -----------------------------
ensure_root() {
  # 确保以root或sudo运行
  if [ "${EUID}" -ne 0 ]; then
    log_error "需要root权限。请使用: sudo bash $0"
    exit 1
  fi
}

append_to_user_bash_profile() {
  local target_home
  target_home="$(eval echo ~${SUDO_USER:-$USER})"
  printf 'if [ -f ~/.bashrc ]; then\n    . ~/.bashrc\nfi\n' >> "${target_home}/.bash_profile"
}

# ----------------------------- 配置文件与常量 -----------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/nfs_config.conf"
DEFAULT_EXPORT_DIR="/home/${SUDO_USER:-$USER}/hy_linux/nfs"
EXPORT_DIR="${EXPORT_DIR:-$DEFAULT_EXPORT_DIR}"

# 允许从配置文件覆盖变量（例如：EXPORT_DIR、ALLOW_CIDR）
[ -f "$CONFIG_FILE" ] && . "$CONFIG_FILE"

get_ipv4() {
  # 功能：获取本机 IPv4 地址（非 lo），优先 iproute2，回退 hostname -I/ifconfig
  # 返回：IPv4 字符串，如 192.168.22.177
  local ip
  if command_exists ip; then
    ip=$(ip -4 -o addr show up | awk '$2!="lo"{split($4,a,"/"); print a[1]; exit}')
  fi
  if [ -z "${ip:-}" ]; then
    ip=$(hostname -I 2>/dev/null | awk '{print $1}')
  fi
  if [ -z "${ip:-}" ] && command_exists ifconfig; then
    ip=$(ifconfig | awk 'match($0, /inet ([0-9.]+)/, a) && a[1]!="127.0.0.1" {print a[1]; exit}')
  fi
  if ! echo "$ip" | grep -Eq '^([0-9]{1,3}\.){3}[0-9]{1,3}$'; then
    log_error "无法获取本机IPv4地址"
    exit 1
  fi
  echo "$ip"
}

cidr_24_from_ip() {
  # 功能：基于 IPv4 生成固定 /24 网段
  # 参数：IPv4 地址
  # 返回：CIDR，如 192.168.22.0/24
  local ip="$1" o1 o2 o3 o4
  IFS='.' read -r o1 o2 o3 o4 <<< "$ip"
  echo "${o1}.${o2}.${o3}.0/24"
}

# ----------------------------- 安装NFS服务器 -----------------------------
# 说明：安装并启用nfs-kernel-server服务
install_nfs_server() {
  # 功能：安装并启用 nfs-kernel-server（幂等）
  log_info "检查 nfs-kernel-server 是否已安装"
  if dpkg -s nfs-kernel-server >/dev/null 2>&1; then
    log_info "nfs-kernel-server 已安装，跳过安装步骤"
  else
    log_info "安装 nfs-kernel-server"
    apt-get update -y
    apt-get install -y nfs-kernel-server
    dpkg -s nfs-kernel-server >/dev/null 2>&1 || { log_error "nfs-kernel-server 安装失败"; exit 1; }
  fi

  log_info "设置 nfs-kernel-server 开机自启"
  systemctl enable nfs-kernel-server
  
  log_info "启动 nfs-kernel-server 服务"
  systemctl start nfs-kernel-server
}

# ----------------------------- 配置导出目录 -----------------------------
# 说明：创建 ~/hy_linux/nfs 并配置 /etc/exports
configure_exports() {
  # 功能：创建导出目录，写入 /etc/exports 并应用导出
  # 参数：无（使用全局 EXPORT_DIR/ALLOW_CIDR）
  local export_dir="$EXPORT_DIR"

  log_info "创建导出目录: $export_dir"
  mkdir -p "$export_dir"
  chmod 777 "$export_dir"

  # 简化：仅根据本机IP生成固定/24网段
  local ip="$(get_ipv4)"
  log_info "本机IP: $ip"
  local detected_cidr
  if [ -n "${ALLOW_CIDR:-}" ]; then
    detected_cidr="$ALLOW_CIDR"
    log_info "使用配置文件指定网段: $detected_cidr"
  else
    detected_cidr="$(cidr_24_from_ip "$ip")"
    log_info "目标网段: $detected_cidr"
  fi

  local exports_line="$export_dir ${detected_cidr}(rw,sync,no_subtree_check,no_root_squash)"

  log_info "备份 /etc/exports 到 /etc/exports.bak"
  cp -f /etc/exports /etc/exports.bak 2>/dev/null || true

  if grep -Fq "$export_dir" /etc/exports 2>/dev/null; then
    log_warn "/etc/exports 已包含该目录的配置，尝试更新为: $exports_line"
    sed -i "\#^$export_dir #d" /etc/exports || true
  fi

  echo "$exports_line" >> /etc/exports

  log_info "应用导出配置: exportfs -a"
  exportfs -a

  log_info "当前导出:"
  exportfs -v || true
}

# ----------------------------- 验证服务 -----------------------------
# 说明：打印服务状态、导出列表、自动启动状态
verify_service() {
  # 功能：输出服务状态、导出列表、自启动与活跃状态
  log_info "检查服务状态: systemctl status nfs-kernel-server"
  systemctl status nfs-kernel-server --no-pager || true

  log_info "导出列表: showmount -e localhost"
  if command_exists showmount; then
    showmount -e localhost || true
  else
    log_warn "未找到 showmount（来自 nfs-common），可安装后验证: apt-get install -y nfs-common"
  fi

  log_info "验证自动启动: systemctl is-enabled nfs-kernel-server"
  systemctl is-enabled nfs-kernel-server || true

  log_info "重启服务以验证运行状态"
  systemctl restart nfs-kernel-server
  systemctl is-active nfs-kernel-server || true
}

# ----------------------------- 主流程 -----------------------------
main() {
  # 脚本入口：统一调度依赖、安装、配置与验证
  log_info "NFS服务器自动配置脚本开始"
  if [ "${TEST_ONLY:-0}" != "1" ]; then
    ensure_root
  fi
  append_to_user_bash_profile
  ensure_dependencies

  if [ "${TEST_ONLY:-0}" = "1" ]; then
    local ip cidr
    ip="$(get_ipv4)"
    cidr="$(cidr_24_from_ip "$ip")"
    log_info "TEST_ONLY 本机IP: $ip"
    log_info "TEST_ONLY 目标网段: $cidr"
    exit 0
  else
    install_nfs_server
    configure_exports
    verify_service
  fi

  local export_dir="$EXPORT_DIR"
  local srv_ip
  srv_ip="$(hostname -I | awk '{print $1}')"
  log_info "配置完成。客户端挂载示例："
  echo "  sudo mount -t nfs -o nolock ${srv_ip}:${export_dir} /mnt/nfs"
  log_info "若要验证重启后服务自动启动，请执行: sudo reboot （脚本不自动重启）"
  {
    local target_home
    target_home="$(eval echo ~${SUDO_USER:-$USER})"
    [ -f "${target_home}/.bash_profile" ] && . "${target_home}/.bash_profile"
  }
  
}

main "$@"
