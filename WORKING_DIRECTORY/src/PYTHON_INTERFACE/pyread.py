

"""
This program lets you access and write to shared memory variables from c
Presently lets you read/write integers,floats
characters are fairly straightforward, an ord() is enough to read it in python 

"""
import sysv_ipc as shm
import time
import struct


def to_float(b): # Convert binary string to a float
    x=''
    for i in b:
        y=''
        if ord(i)==0:
            y='00000000' #8 bits
        else:
            y=format(ord(i),'08b')
        x+=y
    bf = int_to_bytes(int(x, 2), 4)  # 8 bytes needed for IEEE 754 binary64
    return struct.unpack('>f', bf)[0]


def int_to_bytes(n, minlen=0): # Int/long to byte string
    nbits = n.bit_length() + (1 if n < 0 else 0)  # plus one for any sign bit
    nbytes = (nbits+7) // 8  # number of whole bytes
    b = bytearray()
    for _ in range(nbytes):
        b.append(n & 0xff)
        n >>= 8
    if minlen and len(b) < minlen:  # zero pad?
        b.extend([0] * (minlen-len(b)))
    return bytearray(reversed(b))  # high bytes first



def float_to_bin(f): # Convert a float into a binary string
    ba = struct.pack('>f', f)
    ba = bytearray(ba)
    s = ''.join('{:08b}'.format(b) for b in ba)
    return s[:-1] + s[0] # strip all leading zeros except for last



def frombits(bits):
    chars = []
    for b in range(len(bits) / 8):
        byte = bits[b*8:(b+1)*8]
        chars.append(chr(int(''.join([str(bit) for bit in byte]), 2)))
    return ''.join(chars)




def load_mem(key,size=1):    # loads from the shared memory onto a sysv_ipc.sharedmemory object
    return shm.SharedMemory(key,size)

def read_float(mem):
    return to_float(mem[::-1])  ## converts read part to float, and returns

def read_int(mem):
    val=0
    for i in range(len(mem)):
        val+=pow(256,i)*ord(mem[i])
    return val

def write_mem(mem,f):
    mem.write(frombits(float_to_bin(f))[::-1])  ## converts float to binary string, then binary string to actual ascii

def attach(key,size):
    mem = shm.SharedMemory(key=key,flags=shm.IPC_CREAT,size=size)
    return mem
    
"""
Testing part
while True:
    f1 = read_mem(memory1)
    f2 = read_mem(memory2)
    f3 = read_mem(memory3)
    print f1,f2,f3
    f1+=1
    write_mem(memory1,f1)
    write_mem(memory2,f2)
    write_mem(memory3,f3)
    time.sleep(0.1)
"""
