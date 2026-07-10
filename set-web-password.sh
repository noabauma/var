#!/usr/bin/env bash
# Change the slowmo-cam web login (user: billiardino).
#
# The password the browser checks is the HASH in /etc/nginx/slowmo.htpasswd —
# ~/slowmo-cam-password.txt is only a plain-text note for the humans. This
# script updates both and reloads nginx, so the new password works instantly.
set -euo pipefail

read -rsp "New password for web user 'billiardino': " pw; echo
read -rsp "Repeat: " pw2; echo
[ "$pw" = "$pw2" ] || { echo "passwords do not match" >&2; exit 1; }
[ "${#pw}" -ge 8 ] || { echo "use at least 8 characters" >&2; exit 1; }

hash=$(printf '%s' "$pw" | openssl passwd -apr1 -stdin)
printf 'billiardino:%s\n' "$hash" | sudo tee /etc/nginx/slowmo.htpasswd >/dev/null
sudo chown root:www-data /etc/nginx/slowmo.htpasswd
sudo chmod 640 /etc/nginx/slowmo.htpasswd
sudo systemctl reload nginx

printf '%s\n' "$pw" > ~/slowmo-cam-password.txt
chmod 600 ~/slowmo-cam-password.txt
echo "done — new password is active (note updated in ~/slowmo-cam-password.txt)"
