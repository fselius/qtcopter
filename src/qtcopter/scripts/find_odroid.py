#!/usr/bin/env python
'''
Find MAC in local network. This will only work if we are in the same network.
GOOD:
- at home
- same room ethernet in the technion
BAD:
- searching from wifi when MAC is connected physically.


'''

import os
from ip_scan import get_all_mac_ip

MACS = [
    # TODO: add more MAC addresses here
    '36:e6:6a:0e:97:b1',
]

if os.geteuid() != 0:
    print 'warning: not running as root, may not work'

for mac, ip in get_all_mac_ip():
    #print '%r, %r' % (mac, ip)
    for pattern in MACS:
        if mac.startswith(pattern):
            print mac, ip
