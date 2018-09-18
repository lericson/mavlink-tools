import sys
import time
from os import path
from argparse import ArgumentParser

from pymavlink import mavutil

#FTP_HEADER = '=HBBBBBxI'
#'seq_number session opcode size req_opcode burst_complete offset'
#
#(OP_NONE, OP_TERMINATE_SESSION, OP_RESET_SESSION, OP_LIST_DIRECTORY,
# OP_OPEN_FILE_RO, OP_READ_FILE, OP_CREATE_FILE, OP_WRITE_FILE, OP_REMOVE_FILE,
# OP_CREATE_DIRECTORY, OP_OPEN_FILE_WO, OP_TRUNCATE_FILE, OP_RENAME,
# OP_CALC_FILE_CRC32, OP_BURST_READ_FILE) = range(15)
#OP_ACK, OP_Nack = 128, 129
#
#seq_number = 0
#session = 0
#seq_number += 1
#opcode = OP_OPEN_FILE_RO
#filepath = b'/bootlog.txt'
#size = len(filepath)
#req_opcode = 0
#burst_complete = False
#padding = 0
#offset = 0
#
#ftp_payload = struct.pack(FTP_HEADER, seq_number, session, opcode, size, req_opcode, burst_complete, offset) + filepath

parser = ArgumentParser()
parser.add_argument('device')
parser.add_argument('--baud', type=int, default=115200)
parser.add_argument('--update', '-u', action='store_true', help='download '
                    'already-existing files again')

args = parser.parse_args()

def info(*a, **k):
    print(*a, **k, file=sys.stderr)

info(f'  connecting to {args.device} at baud {args.baud}', end='\r', flush=True)
master = mavutil.mavlink_connection(args.device, baud=args.baud, robust_parsing=True)
master.wait_heartbeat()
time.sleep(0.5)
info(f'\033[2K+ connected to {args.device} at baud {args.baud} using {master.WIRE_PROTOCOL_VERSION}')

info(f'  requesting log entries', end='\r', flush=True)
master.mav.log_request_list_send(master.target_system, master.target_component, 0, 0xffff)
logs = []
while True:
    m = master.recv_match(blocking=True, type='LOG_ENTRY', timeout=5)
    if m is None:
        master.mav.log_request_list_send(master.target_system, master.target_component, 0, 0xffff)
        continue
    info(f'  received {len(logs)} log entries [{len(logs)*100//(m.num_logs+1):d}%]', end='\r', flush=True)
    logs.append(m)
    if m.id == m.last_log_num:
        break
info(f'\033[2K+ found {len(logs)} log entries')

for log in logs:
    logfn = f'log_{log.id}.ulg'
    if path.exists(logfn) and not args.update:
        info(f'- log {log.id} already downloaded, skipping')
        continue
    elif log.size == 0:
        info(f'- log {log.id} is empty, skipping')
        continue
    ofs = 0
    master.mav.log_request_data_send(master.target_system, master.target_component,
                                     log.id, ofs, log.size - ofs)
    with open(logfn, 'wb') as f:
        while ofs < log.size:
            info(f'\033[2K  downloading log {log.id} ({log.size/1024./1024:.2f} MiB) [{ofs*100//log.size:d}%]', end='\r', flush=True)
            m = master.recv_match(blocking=True, type='LOG_DATA', timeout=5)
            if m is None:
                master.mav.log_request_data_send(master.target_system, master.target_component,
                                                 log.id, ofs, log.size - ofs)
                continue
            assert m.id == log.id
            assert m.ofs == ofs
            ofs = m.ofs + m.count
            f.write(bytes(m.data[:m.count]))
        info(f'\033[2K+ downloaded log {log.id} ({log.size/1024./1024:.2f} MiB)')
    #master.mav.log_request_end_send(master.target_system, master.target_component)
