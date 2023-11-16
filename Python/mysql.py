import pymysql

def connectMysql():
    db = pymysql.connect(host = "localhost", user = "root", password = "123456", database = "cet_db", port=3306)

    return db

def find(sql):
    # SQL 查询语句
    db = connectMysql()
    try:
        # 执行SQL语句
        cursor = db.cursor()
        cursor.execute(sql)
        # 获取所有记录列表
        results = cursor.fetchall()

        # print(str(results))
        return results
        # for row in results
    except Exception as err:
        print(err)
    db.close()
    return 0

def add(sql):
    # 使用cursor()方法获取操作游标
    db = connectMysql()
    cursor = db.cursor()
    # SQL 插入语句
    try:
        # 执行sql语句
        cursor.execute(sql)
        # 执行sql语句
        db.commit()
        print("保存成功")
    except Exception as e:
        print(e)
        # 发生错误时回滚
        db.rollback()
        print("错误")
    # 关闭数据库连接
    db.close()