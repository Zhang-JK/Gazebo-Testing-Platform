import re

# Read the file
def decode(line):
    if (line == '' or line.startswith('#')):
        return 
    line.split(',')
    m1 = int(re.search(r"([0-9]+)", line.split(',')[0]).group(0))
    m2 = int(re.search(r"([0-9]+)", line.split(',')[1]).group(0))
    return (m1, m2)

COMMAND_FILE = './movement.txt'
with open(COMMAND_FILE) as file:
    lines = [decode(line.rstrip()) for line in file]

print(lines)