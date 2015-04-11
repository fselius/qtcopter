#!/usr/bin/expect -f
spawn ssh-keygen -t rsa -C "qtcopter Virtual machine" -f ssh/id_rsa
expect "Generating public/private rsa key pair."
expect "Enter passphrase (empty for no passphrase): "
send "SecretQtcopterPassphrase\n";
expect "Enter same passphrase again: "
send "SecretQtcopterPassphrase\n";
interact
