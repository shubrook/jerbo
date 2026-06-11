import socket
import time
import numpy as np
import pandas as pd
import urllib
from sqlalchemy import create_engine
import threading

# --- Configuration ---
HOST = ''
PORT = 12345
PI_HOST = '192.168.0.164'
SAMPLE_WINDOW = 256
NUM_MICS = 4

# 1. Establish the engine OUTSIDE the loop
conn_str = (
    r'Driver={ODBC Driver 18 for SQL Server};'
    r'Server=localhost\MSSQLSERVER01;'
    r'Database=jerbo;'
    r'Trusted_Connection=Yes;'
    r'TrustServerCertificate=Yes;'
)
quoted_conn_str = urllib.parse.quote_plus(conn_str)
engine = create_engine(f'mssql+pyodbc:///?odbc_connect={quoted_conn_str}',
                       fast_executemany=True) # Boosts performance for bulk inserts

# --- Socket Setup ---
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
print(f"Listening on port {PORT}...")

def write_thread():

    conn, addr = s.accept()
    print('Connected by', addr)

    # Expected size in bytes: (SAMPLE_WINDOW * NUM_MICS * 2) * 8  # ts + value per sample
    expected_bytes = (SAMPLE_WINDOW * NUM_MICS * 2) * 8

    while True:
        # 2. Use a loop to ensure you receive the FULL buffer
        data = b''
        while len(data) < expected_bytes:
            packet = conn.recv(expected_bytes - len(data))
            if not packet: break
            data += packet

        if not data: break

        batch = np.frombuffer(data, dtype=np.float64).reshape(NUM_MICS, SAMPLE_WINDOW, 2)  # mics x samples x (ts, value)

        mic_names = ['left', 'right', 'top', 'bottom']

        # 3. Efficient DataFrame creation
        df = pd.DataFrame()
        for ch in range(NUM_MICS):
            mic_df = pd.DataFrame({
                'timestamp': batch[ch, :, 0],
                'mic': [mic_names[ch]] * SAMPLE_WINDOW,
                'volume': batch[ch, :, 1]
            })
            df = pd.concat([df, mic_df], ignore_index=True)

        # 4. Write to SQL using the existing engine
        df.to_sql('telemetry', con=engine, if_exists='append', index=False)

def read_thread():
    PI_HOST = '192.168.0.164'  # Pi IP
    PI_PORT = 12346  # Reverse port

    pi_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pi_s.connect((PI_HOST, PI_PORT))
    while True:
        sqlcmd = f"select top 1 *, current_timestamp as current_ts from vw_clumped_readings with (nolock) order by ts_round desc"

        df = pd.read_sql(sqlcmd, con = engine)
        if df is not None:

            pan_angle = df['mean_pan_angle'][0]
            tilt_angle = df['mean_tilt_angle'][0]
            current_ts = df['current_ts'][0].timestamp()
            angles = np.array([pan_angle, tilt_angle, current_ts], dtype=np.float64)
            pi_s.sendall(angles.tobytes())
            time.sleep(0.05)


threading.Thread(target=write_thread).start()
threading.Thread(target=read_thread).start()
