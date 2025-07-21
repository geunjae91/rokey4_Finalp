import sqlite3
import os

# 현재 스크립트 위치 기준으로 database.db 생성
DB_PATH = os.path.join(os.path.dirname(__file__), 'database.db')

def init_db():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # users 테이블 생성
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS users (
            id TEXT PRIMARY KEY,
            password TEXT NOT NULL
        )
    ''')

    # admin 계정 추가 
    cursor.execute("INSERT OR IGNORE INTO users (id, password) VALUES (?, ?)", ('admin', '1234'))
    cursor.execute("INSERT OR IGNORE INTO users (id, password) VALUES (?, ?)", ('user', '1234'))
    
    # order 페이지 장바구니 수량 카운트
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS cart_items (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            product_name TEXT NOT NULL,
            quantity INTEGER DEFAULT 1
        )
    ''')
    cursor.execute('''
       CREATE TABLE IF NOT EXISTS products (
            category TEXT,
            product_name TEXT PRIMARY KEY,
            product_id TEXT,
            price TEXT,
            stock INTEGER,
            location TEXT
        );            
     ''')
    cursor.execute("INSERT OR IGNORE INTO products VALUES (?, ?, ?, ?, ?, ?)", ('Health', '단백질 쉐이크', 'PRT-001', '$3.00', 2, 'Zone A-1'))
    cursor.execute("INSERT OR IGNORE INTO products VALUES (?, ?, ?, ?, ?, ?)", ('Stationery', '컬러 테이프', 'TAPE-002', '$8.00', 2, 'Zone A-2'))
    cursor.execute("INSERT OR IGNORE INTO products VALUES (?, ?, ?, ?, ?, ?)", ('Toy', 'RC카', 'RC-003', '$29.99', 2, 'Zone A-3'))
    cursor.execute("INSERT OR IGNORE INTO products VALUES (?, ?, ?, ?, ?, ?)", ('Electronics', '공유기', 'NET-004', '$50.00', 2, 'Zone A-4'))
    
    # 주문 테이블
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS orders_history (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            phone TEXT NOT NULL,
            address TEXT NOT NULL,
            product_name TEXT NOT NULL,
            product_ea TEXT NOT NULL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')

        
    
    conn.commit()
    conn.close()
    print("데이터베이스 초기화 완료!")


if __name__ == "__main__":
    init_db()
