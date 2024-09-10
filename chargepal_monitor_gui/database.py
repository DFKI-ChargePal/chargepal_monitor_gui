#!/usr/bin/env python3
import sqlite3
import config as cfg
from typing import List, Dict
    
# Create tables and insert data into the database
def initialize_database():
    conn_ldb = sqlite3.connect(cfg.ldb_path)
    cursor_ldb = conn_ldb.cursor()
    cursor_ldb.execute(
        """
        CREATE TABLE IF NOT EXISTS robot_info (
            name TEXT,
            robot_location TEXT,
            current_job_id INTEGER,
            current_job TEXT,
            ongoing_action TEXT,
            previous_action TEXT,
            cart_on_robot TEXT,
            job_status TEXT,
            availability INTEGER,
            robot_charge FLOAT,
            error_count INTEGER
        )
        """
    )
    cursor_ldb.execute(
        """
        CREATE TABLE IF NOT EXISTS cart_info (
            name TEXT,
            cart_location TEXT,
            robot_on_cart TEXT,
            plugged TEXT,
            action_state TEXT,
            error_count INTEGER
        )
        """
    )

    cursor_ldb.execute(
        """
        CREATE TABLE IF NOT EXISTS env_info (
            name TEXT,
            value TEXT,
            count INTEGER
        )
        """
    )
    
    conn_ldb.commit()
    conn_ldb.close()
    

def clear_database():
    conn_ldb = sqlite3.connect(cfg.ldb_path)
    cursor_ldb = conn_ldb.cursor()
    cursor_ldb.execute("DELETE FROM orders_in")
    cursor_ldb.execute("DELETE FROM robot_info")
    cursor_ldb.execute("DELETE FROM cart_info")
    cursor_ldb.execute("DELETE FROM env_info")
    conn_ldb.commit()
    conn_ldb.close()
    

def insert_robot_data(data):
    conn = sqlite3.connect(cfg.ldb_path)
    cursor = conn.cursor()
    cursor.execute("INSERT INTO robot_info (name,robot_location,current_job_id,current_job, ongoing_action, previous_action, cart_on_robot,job_status,availability,robot_charge,error_count) VALUES (?,?,?,?,?,?,?,?,?,?,?)",data,)
    conn.commit()
    conn.close()


def insert_cart_data(data):
    conn = sqlite3.connect(cfg.ldb_path)
    cursor = conn.cursor()
    cursor.execute(
        "INSERT INTO cart_info (name,cart_location, robot_on_cart, plugged,action_state, error_count) VALUES (?,?,?,?,?,?)",
        data,
    )
    conn.commit()
    conn.close()


def insert_env_info(data):
    conn = sqlite3.connect(cfg.ldb_path)
    cursor = conn.cursor()
    cursor.execute(
        "INSERT INTO env_info (name, value, count) VALUES (?,?,?)",
        data,
    )
    conn.commit()
    conn.close()

# Connect to SQLite database
def get_db_connection(db_path):
    return sqlite3.connect(db_path)

def fetch_table_names(db_path:str)-> List[str]:
    conn = get_db_connection(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
    table_names = [row[0] for row in cursor.fetchall()]
    conn.close()  # Don't forget to close the connection
    return table_names

# Fetch all data from a specific table
def fetch_table_data(db_path:str, table_name: str):
    conn = get_db_connection(db_path)
    cursor = conn.cursor()
    cursor.execute(f"SELECT * FROM {table_name}")
    rows = cursor.fetchall()
    
    # Get column names
    columns = [description[0] for description in cursor.description]
    conn.close()  # Always close the connection after use
    return columns, rows
