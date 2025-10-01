Installation
- `bashrc`s just go in root as `~/.bashrc` (don't keep @sc or @nand)
- `start_buggy.sh` is copied into ~: `cp start_buggy.sh ~`
- same with `stop_buggy.sh`
- `buggy@?.service` is installed into systemd as `buggy.service` (no @sc or @nand)
- optional: install motd at /etc/motd

Note: `bashrc` and `start_buggy.sh` are the same for both buggies,
and should be maintained to be independent of the buggy. The systemd
units are NOT the same.

