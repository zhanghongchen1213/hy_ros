import numpy as np
import os

def analyze_channels(path, expected_h, expected_w, channels=32):
    if not os.path.exists(path):
        return
    
    data = np.fromfile(path, dtype=np.int8)
    grid_size = expected_h * expected_w
    expected_size = channels * grid_size
    
    if data.size != expected_size:
        print(f"File {os.path.basename(path)} size mismatch. Expected {expected_size}, got {data.size}")
        return

    # Assume NCHW for now (since we want to see per-channel stats)
    # Even if it's NHWC, reshaping to (C, H, W) will mix data, but Min/Max/Mean per 'chunk' might tell us something if we assume NCHW.
    # Actually, if it's NHWC, we should reshape to (H, W, C).
    
    # Let's try both assumptions for statistics.
    
    print(f"\nAnalyzing {os.path.basename(path)} (Assuming {channels} channels)...")
    
    # Assumption 1: NCHW (planar)
    # Channel 0 is the first grid_size elements.
    print("  [Assumption: NCHW]")
    for c in range(min(5, channels)): # Check first 5 channels
        start = c * grid_size
        end = start + grid_size
        c_data = data[start:end]
        print(f"    Channel {c}: Min={c_data.min()}, Max={c_data.max()}, Mean={c_data.mean():.2f}, Std={c_data.std():.2f}")

    # Assumption 2: NHWC (interleaved)
    # Channel 0 is every C-th element.
    print("  [Assumption: NHWC]")
    for c in range(min(5, channels)):
        c_data = data[c::channels]
        print(f"    Channel {c}: Min={c_data.min()}, Max={c_data.max()}, Mean={c_data.mean():.2f}, Std={c_data.std():.2f}")

def summarize_pair(dump_dir, a, b, h, w):
    pa = os.path.join(dump_dir, f"output_{a}.bin")
    pb = os.path.join(dump_dir, f"output_{b}.bin")
    da = np.fromfile(pa, dtype=np.int8)
    db = np.fromfile(pb, dtype=np.int8)
    grid = h * w
    print(f"\nPair ({a},{b}) @ {h}x{w}")
    print(f"  A: size={da.size}, mean={da.mean():.2f}, std={da.std():.2f}")
    print(f"  B: size={db.size}, mean={db.mean():.2f}, std={db.std():.2f}")
    a0 = da[:grid]
    b0 = db[:grid]
    print(f"  A[0] min={a0.min()} max={a0.max()} mean={a0.mean():.2f} std={a0.std():.2f}")
    print(f"  B[0] min={b0.min()} max={b0.max()} mean={b0.mean():.2f} std={b0.std():.2f}")
    print("  Recommendation: treat first as objectness and second as class")

def main():
    dump_dir = "/home/k/hy_linux/nfs/hy_ros/rknn_dump/"
    print("--- Stride 8 (80x80) ---")
    analyze_channels(os.path.join(dump_dir, "output_1.bin"), 80, 80, 32)
    analyze_channels(os.path.join(dump_dir, "output_2.bin"), 80, 80, 32)
    summarize_pair(dump_dir, 1, 2, 80, 80)
    print("\n--- Stride 16 (40x40) ---")
    analyze_channels(os.path.join(dump_dir, "output_4.bin"), 40, 40, 32)
    analyze_channels(os.path.join(dump_dir, "output_5.bin"), 40, 40, 32)
    summarize_pair(dump_dir, 4, 5, 40, 40)

if __name__ == "__main__":
    main()
