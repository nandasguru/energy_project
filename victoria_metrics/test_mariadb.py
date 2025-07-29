import mariadb

try:
    conn = mariadb.connect(
        user = "testuser",
        password = "testpass",
        host = "localhost",
        database = "testdb"
    )
    cursor = conn.cursor()

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS users (
            id INT PRIMARY KEY AUTO_INCREMENT,
            name VARCHAR(100)
        )
    """)

    cursor.execute("Insert into users (name) values (?)", ("Alice",))
    conn.commit()

    cursor.execute("SELECT * FROM users")
    for(user_id, name) in cursor:
        print(f"User ID: {user_id}, Name: {name}")

except mariadb.Error as e:
    print(f"MariaDB error: {e}")
finally:
    if conn:
        conn.close()