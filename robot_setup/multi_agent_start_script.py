import os

os.chdir("//")
ip_addresses = open("../../../Users/crl/Downloads/ip_samples.txt", 'r')
for line in ip_addresses:
    os.system("gnome-terminal -- /bin/bash -c 'sshpass -v -p crl1234 ssh ubuntu@{ip}'".format(ip = line))
ip_addresses.close()

