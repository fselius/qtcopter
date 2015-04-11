#!/usr/bin/expect -f
spawn ssh-add ssh/id_rsa
expect "Enter passphrase for ssh/id_rsa:"
send "SecretQtcopterPassphrase\n";
interact
