import os

prio_value = int(input('Please choose a priority between 1 and 99\n'))

# -> Set program priority in scheduler POSIX
sp_max = os.sched_get_priority_max(os.SCHED_FIFO)
#prio_value = 99
sp     = os.sched_param(prio_value)             # range 1 (low) - 99 (high)
sched_type = os.SCHED_RR #os.SCHED_FIFO #          # Choose from FIFO or RR
if sched_type == 1:
    sched = 'First In First Out'
else:
    sched = 'Round Robin'
try:
    # => Use FIFO (First In First Out) or RR (Round Robin)
    os.sched_setscheduler(0, sched_type, sp)   # FIFO or RR are real-time policies (linux)
    print("Priority set to", sched, "value ", str(prio_value))
    input("Press a key to acknoledge")
except PermissionError:
    print("\nCan't set that priority!!! Exiting\n")
    input("Press a key to acknoledge")
    quit

n1, n2 = 0, 1
count, nterms = 0, 1
while count < nterms:
       print(n1, '\n')
       nth = n1 + n2
       # update values
       n1 = n2
       n2 = nth
       count += 1
       nterms += 1
