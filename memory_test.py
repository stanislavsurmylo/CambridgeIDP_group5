import os
s = os.statvfs('/')                 # (bsize, frsize, blocks, bfree, ...)
total = s[0] * s[2]
free  = s[0] * s[3]
print('FS total:', total//1024, 'KB, free:', free//1024, 'KB')

# list files with sizes
for name, type, inode, size in os.ilistdir('/'):
    if type == 0x8000:  # file
        print(f'{name:20s} {size:6d} B')
