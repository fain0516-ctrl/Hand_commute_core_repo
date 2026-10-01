"""실행: python3 -m comm_core [--config config/comm_core.example.json]"""

import argparse
import logging
import time

from .config import load_config
from .core import CommCore


def main() -> None:
    p = argparse.ArgumentParser(description="Pi 5 comm core (team UDP <-> Pico 2 TCP)")
    p.add_argument("--config", help="JSON config path (기본값 사용 시 생략)")
    p.add_argument("--stats-every", type=float, default=5.0, help="상태 로그 주기 (s), 0 이면 끔")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    core = CommCore(load_config(args.config))
    core.start()
    log = logging.getLogger("comm_core")
    try:
        while True:
            chunk = args.stats_every if args.stats_every > 0 else 3600.0
            core.run(duration_s=chunk)
            if args.stats_every > 0:
                log.info("stats %s", core.snapshot())
    except KeyboardInterrupt:
        pass
    finally:
        core.close()
        time.sleep(0.05)


if __name__ == "__main__":
    main()
