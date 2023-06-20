from flask import Flask, render_template, request, url_for, flash, redirect
import pymongo, json
import os
from dotenv import load_dotenv

load_dotenv()
DATABASE_URL ='mongodb+srv://RobotImprov:'+os.environ.get("password")+'@cluster0.r2dn6t9.mongodb.net/?retryWrites=true&w=majority'
client = pymongo.MongoClient('mongodb://localhost:27017')
#client = pymongo.MongoClient(DATABASE_URL)
db = client.RobotImprov
ans = db.Answers

app = Flask(__name__)

@app.route("/")
def index():
    return render_template('homepage.html')

@app.route('/start', methods=['POST'])
def start():
    return render_template('evaluation.html')


@app.route('/handle_answer', methods=['POST'])
def handle_answer():
    answer = request.form['answerinput']
    answer = json.loads(answer)
    
    print("here is the dict: " )
    print(answer)

    print("here is the labels: " + str(answer["labels"]))
    os.chdir(os.getcwd()+'/VideoLabeling')
    with open('labels.json', 'w') as f:
        json.dump(answer, f)
    return render_template('thanks.html')

if __name__ == '__main__':
    app.run(debug=True)
    
