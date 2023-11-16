import json

from flask import Flask
from flask import request
from flask import jsonify
import  mysql
import random
import requests

app = Flask(__name__)






# 获取单词
@app.route('/getWord', methods=['GET', 'POST'])
def index():
    randint = random.randint(1, 4000)
    res = mysql.find("select * from cet4zx where id = " + str(randint));
    print(res)
    return jsonify({"id": res[0][0], "word": res[0][1], "sent": res[0][2], "chinese": res[0][3], "code": 200})


# 获取天气
@app.route('/getWeather', methods=['GET', 'POST'])
def getWeather():
  # 心知天气的接口，需要取获取Key
    res = requests.get("https://api.seniverse.com/v3/weather/daily.json?key=***&location=zhuhai&language=zh-Hans&unit=c&start=-1&days=5")
    json_data = json.loads(res.text)
    print(json_data)

    return jsonify({"city": json_data['results'][0]['location']['name'], "code_day": int(json_data['results'][0]['daily'][0]['code_day']), "text_night": json_data['results'][0]['daily'][0]['text_night']})
    #return jsonify({"city": "珠海", "code_day": 1, "text_night": "阵雨"})



if __name__ == '__main__':
    app.json.ensure_ascii = False
    app.run(host='0.0.0.0', port=5001, debug=True)