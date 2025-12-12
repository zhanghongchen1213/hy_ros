#!/usr/bin/env python3

# Real-time speech recognition from a microphone with sherpa-onnx Python API
# with endpoint detection.
#
# Please refer to
# `https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html`
# to download pre-trained models

import argparse
import sys
from pathlib import Path
import signal

try:
    import sounddevice as sd
except ImportError:
    print("Please install sounddevice first. You can use")
    print()
    print("  pip install sounddevice")
    print()
    print("to install it")
    sys.exit(-1)

import sherpa_onnx


def assert_file_exists(filename: str):
    assert Path(filename).is_file(), (
        f"{filename} does not exist!\n"
        "Please refer to "
        " `https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html`  to download it"
    )


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    model_dir = '/opt/sherpa-onnx/sherpa-onnx-rk3588-streaming-zipformer-small-bilingual-zh-en-2023-02-16'

    parser.add_argument(
        "--tokens",
        type=str,
        default=f"{model_dir}/tokens.txt",
        help="Path to tokens.txt",
    )

    parser.add_argument(
        "--encoder",
        type=str,
        default=f"{model_dir}/encoder.rknn",
        help="Path to the encoder model",
    )

    parser.add_argument(
        "--decoder",
        type=str,
        default=f"{model_dir}/decoder.rknn",
        help="Path to the decoder model",
    )

    parser.add_argument(
        "--joiner",
        type=str,
        default=f"{model_dir}/joiner.rknn",
        help="Path to the joiner model",
    )

    parser.add_argument(
        "--decoding-method",
        type=str,
        default="greedy_search",
        help="Valid values are greedy_search and modified_beam_search",
    )

    parser.add_argument(
        "--provider",
        type=str,
        default="rknn",
        help="Valid values: cpu, cuda, coreml, rknn",
    )

    parser.add_argument(
        "--hotwords-file",
        type=str,
        default="",
        help="""
        The file containing hotwords, one words/phrases per line, and for each
        phrase the bpe/cjkchar are separated by a space. For example:

        ▁HE LL O ▁WORLD
        你 好 世 界
        """,
    )

    parser.add_argument(
        "--hotwords-score",
        type=float,
        default=1.5,
        help="""
        The hotword score of each token for biasing word/phrase. Used only if
        --hotwords-file is given.
        """,
    )

    parser.add_argument(
        "--blank-penalty",
        type=float,
        default=0.0,
        help="""
        The penalty applied on blank symbol during decoding.
        Note: It is a positive value that would be applied to logits like
        this `logits[:, 0] -= blank_penalty` (suppose logits.shape is
        [batch_size, vocab] and blank id is 0).
        """,
    )

    parser.add_argument(
        "--hr-lexicon",
        type=str,
        default="",
        help="If not empty, it is the lexicon.txt for homophone replacer",
    )

    parser.add_argument(
        "--hr-rule-fsts",
        type=str,
        default="",
        help="If not empty, it is the replace.fst for homophone replacer",
    )

    return parser.parse_args()


def create_recognizer(args):
    assert_file_exists(args.encoder)
    assert_file_exists(args.decoder)
    assert_file_exists(args.joiner)
    assert_file_exists(args.tokens)
    # Please replace the model files if needed.
    # See `https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html`
    # for download links.
    recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        num_threads=1,
        sample_rate=16000,
        feature_dim=80,
        enable_endpoint_detection=True,
        rule1_min_trailing_silence=2.4,
        rule2_min_trailing_silence=1.2,
        rule3_min_utterance_length=0.0,  # Modified to 0.0 to match our debugging finding
        decoding_method=args.decoding_method,
        provider=args.provider,
        hotwords_file=args.hotwords_file,
        hotwords_score=args.hotwords_score,
        blank_penalty=args.blank_penalty,
        hr_rule_fsts=args.hr_rule_fsts,
        hr_lexicon=args.hr_lexicon,
    )
    return recognizer


def main():
    args = get_args()

    devices = sd.query_devices()
    if len(devices) == 0:
        print("No microphone devices found")
        sys.exit(0)

    print(devices)
    
    # Auto-select USB PnP Sound Device if available
    target_device_idx = None
    for idx, device in enumerate(devices):
        if "USB PnP Sound Device" in device["name"]:
            target_device_idx = idx
            break
            
    if target_device_idx is not None:
        print(f"\nFound USB microphone at index {target_device_idx}: {devices[target_device_idx]['name']}")
    else:
        # Fallback to default if not found (though user said only USB works)
        target_device_idx = sd.default.device[0]
        print(f"\nUSB microphone not found. Using default device index {target_device_idx}: {devices[target_device_idx]['name']}")

    print(f"Using provider: {args.provider}")
    print(f"Loading models from: {args.encoder}")

    recognizer = create_recognizer(args)
    print("Started! Please speak")

    # The model is using 16 kHz, we use 48 kHz here to demonstrate that
    # sherpa-onnx will do resampling inside.
    sample_rate = 48000
    samples_per_read = int(0.1 * sample_rate)  # 0.1 second = 100 ms

    stream = recognizer.create_stream()

    display = sherpa_onnx.Display()

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nCaught Ctrl + C. Exiting")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)

    try:
        with sd.InputStream(channels=1, dtype="float32", samplerate=sample_rate, device=target_device_idx) as s:
            while True:
                samples, _ = s.read(samples_per_read)  # a blocking read
                samples = samples.reshape(-1)
                
                # Debug: Print max amp occasionally
                max_amp = float(max(abs(samples.min()), abs(samples.max())))
                if max_amp > 0.1:
                     print(f"\rAudio detected: {max_amp:.2f}", end="", flush=True)

                stream.accept_waveform(sample_rate, samples)
                while recognizer.is_ready(stream):
                    recognizer.decode_stream(stream)

                is_endpoint = recognizer.is_endpoint(stream)

                result = recognizer.get_result(stream)
                
                # Adapt for both string and object return types
                if isinstance(result, str):
                    text = result
                else:
                    text = result.text

                if text:
                    if isinstance(result, str):
                        print(f"\rCurrent text: {text}", end="", flush=True)
                    else:
                        display.update_text(result)
                        display.display()

                if is_endpoint:
                    if text:
                        if isinstance(result, str):
                            print(f"\nFinal result: {text}")
                        else:
                            display.finalize_current_sentence()
                            display.display()

                    recognizer.reset(stream)
    except Exception as e:
        print(f"\nError: {e}")

if __name__ == "__main__":
    main()
