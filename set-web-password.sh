#!/usr/bin/env bash
# Change a slowmo-cam web login:  ./set-web-password.sh [biliardino|admin]
#
# 'biliardino' (default) is the shared everyone-login; 'admin' additionally
# unlocks the PageRank algorithm switch and the damping slider. The
# password the browser checks is the HASH in
# /etc/nginx/slowmo.htpasswd — the ~/slowmo-cam*-password.txt files are only
# plain-text notes for the humans. This script updates one user's line
# (keeping the other) and reloads nginx, so the new password works instantly.
set -euo pipefail

user="${1:-biliardino}"
case "$user" in
biliardino | admin) ;;
*)
    echo "usage: $0 [biliardino|admin]" >&2
    exit 1
    ;;
esac

read -rsp "New password for web user '$user': " pw; echo
read -rsp "Repeat: " pw2; echo
[ "$pw" = "$pw2" ] || { echo "passwords do not match" >&2; exit 1; }
[ "${#pw}" -ge 8 ] || { echo "use at least 8 characters" >&2; exit 1; }

f=/etc/nginx/slowmo.htpasswd
hash=$(printf '%s' "$pw" | openssl passwd -apr1 -stdin)
{ sudo grep -v "^$user:" "$f" 2>/dev/null || true
  printf '%s:%s\n' "$user" "$hash"; } | sudo tee "$f.new" >/dev/null
sudo mv "$f.new" "$f"
sudo chown root:www-data "$f"
sudo chmod 640 "$f"
sudo systemctl reload nginx

note=~/slowmo-cam-password.txt
[ "$user" = admin ] && note=~/slowmo-cam-admin-password.txt
printf '%s\n' "$pw" > "$note"
chmod 600 "$note"
echo "done — new password for '$user' is active (note updated in $note)"
