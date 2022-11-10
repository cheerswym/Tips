import os
import re
import time 
import datetime

def iterator_paths(file_path):
    for root , dirs, files in os.walk(file_path):
        for path_name in dirs:
            if re.match("\d+-\d+-\d+", path_name):
                path = root + path_name
                if not path.endswith('/'):
                    path = path + '/'
                iterator_files(path)
    return 0    

def iterator_files(file_path):
    for root , dirs, files in os.walk(file_path):
        for name in files:
            if name.startswith("record."):
                read_ctime_from_file(file_path + name)
    return 0    

def read_ctime_from_file(file_name):
    infos = os.stat(file_name)
    ctime = time.localtime(infos.st_ctime)
    ctime_str = time.strftime("%Y-%m-%d %H:%M:%S", ctime)
    print(file_name + '\t' + ctime_str)

if __name__ == "__main__":
    iterator_paths('./')
    #iterator_files('./')
    #read_ctime_from_file("record.00000")
