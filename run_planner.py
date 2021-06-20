import os

#os.path.p
#exit_code = os.system('wsl ~/CBSH2-RTC/cbs -m ~/CBSH2-RTC/random-32-32-20.map -a random-32-32-20-random-1.scen -o test.csv --outputPaths=paths.txt -k 30 -t 60')
exit_code = os.system('wsl ~/CBSH2-RTC/cbs -m my_map.map -a my_scen.scen -o test.csv --outputPaths=paths2.txt -k 2 -t 60')

if exit_code != 0:
    print('Error running the planner')