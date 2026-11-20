"""Check counts and receive rates in a finalized Stage 1 SQLite bag."""

import argparse
from contextlib import closing
from pathlib import Path
import sqlite3
import sys


def expected_rates(mode: str) -> dict[str, int]:
    camera = "/camera/camera"
    images = {
        "infra1": "image_rect_raw",
        "infra2": "image_rect_raw",
        "color": "image_raw",
        "depth": "image_rect_raw",
        "aligned_depth_to_color": "image_raw",
    }
    rates = {f"{camera}/{stream}/{image}": 30 for stream, image in images.items()}
    rates.update({f"{camera}/{stream}/camera_info": 0 for stream in images})
    rates["/tf_static"] = 0
    if mode == "fused":
        rates[f"{camera}/imu"] = 200
    else:
        rates[f"{camera}/gyro/sample"] = 200
        rates[f"{camera}/accel/sample"] = 0
    return rates


def check_bag(directory: Path, mode: str) -> bool:
    databases = list(directory.glob("*.db3"))
    if not (directory / "metadata.yaml").is_file() or len(databases) != 1:
        raise ValueError("Expected a finalized bag directory with exactly one .db3 file.")

    uri = databases[0].resolve().as_uri() + "?mode=ro"
    with closing(sqlite3.connect(uri, uri=True)) as database:
        summary = {
            name: (count, start, end)
            for name, count, start, end in database.execute(
                """
                SELECT topics.name, COUNT(messages.id),
                       MIN(messages.timestamp), MAX(messages.timestamp)
                FROM topics LEFT JOIN messages ON messages.topic_id = topics.id
                GROUP BY topics.name
                """
            )
        }

    passed = True
    for name, target in expected_rates(mode).items():
        count, start, end = summary.get(name, (0, None, None))
        duration = (end - start) / 1e9 if count else 0
        rate = (count - 1) / duration if duration > 0 else 0
        # The short bench capture allows up to 15% receive-rate variation.
        ok = count > 0 if target == 0 else duration >= 10 and abs(rate - target) <= 0.15 * target
        passed &= ok
        label = "OK" if ok else "FAIL"
        print(f"{label:4} {name:51} {count:5} messages  {rate:6.1f} Hz")
    return passed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("fused", "raw"))
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    try:
        return 0 if check_bag(args.bag, args.mode) else 1
    except (ValueError, OSError, sqlite3.Error) as error:
        print(f"Could not check bag: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
