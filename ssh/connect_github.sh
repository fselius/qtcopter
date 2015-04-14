#!/usr/bin/expect -f
spawn ssh git@github.com
expect "Are you sure you want to continue connecting (yes/no)? "
send "yes\n";
interact
