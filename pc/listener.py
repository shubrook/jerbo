"""
Telemetry logger — receives angle estimates from the Pi and logs them to
SQL Server. This is logging only; the control loop runs entirely on the Pi.

Each record from the Pi is 5 float64s:
    timestamp (unix), pan_deg, tilt_deg, rms, confidence
"""
import socket
import urllib.parse

import numpy as np
import pandas as pd
from sqlalchemy import create_engine

HOST = ''
PORT = 12345
BATCH_ROWS = 50  # insert to SQL every N records (~4 s at 12 blocks/s)

RECORD_FIELDS = ['ts_unix', 'pan_deg', 'tilt_deg', 'rms', 'confidence']
RECORD_BYTES = len(RECORD_FIELDS) * 8

conn_str = (
    r'Driver={ODBC Driver 18 for SQL Server};'
    r'Server=localhost\MSSQLSERVER01;'
    r'Database=jerbo;'
    r'Trusted_Connection=Yes;'
    r'TrustServerCertificate=Yes;'
)
engine = create_engine(
    f'mssql+pyodbc:///?odbc_connect={urllib.parse.quote_plus(conn_str)}',
    fast_executemany=True)


def recv_record(conn):
    data = b''
    while len(data) < RECORD_BYTES:
        packet = conn.recv(RECORD_BYTES - len(data))
        if not packet:
            return None
        data += packet
    return np.frombuffer(data, dtype=np.float64)


def flush(rows):
    df = pd.DataFrame(rows, columns=RECORD_FIELDS)
    df.insert(0, 'ts', pd.to_datetime(df['ts_unix'], unit='s'))
    df.to_sql('doa_telemetry', con=engine, if_exists='append', index=False)
    print(f"logged {len(df)} rows, last pan {df['pan_deg'].iloc[-1]:.1f} "
          f"tilt {df['tilt_deg'].iloc[-1]:.1f}")


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))
s.listen(1)
print(f"Listening on port {PORT}...")

while True:  # outer loop so the Pi can reconnect after a restart
    conn, addr = s.accept()
    print('Connected by', addr)
    rows = []
    try:
        while True:
            record = recv_record(conn)
            if record is None:
                break
            rows.append(record)
            if len(rows) >= BATCH_ROWS:
                flush(rows)
                rows = []
    finally:
        if rows:
            flush(rows)
        conn.close()
        print('Disconnected; waiting for new connection')
