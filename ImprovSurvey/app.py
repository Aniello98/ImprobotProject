from flask import Flask, render_template, request, url_for, flash, redirect
import pymongo, json
import os
from dotenv import load_dotenv
import threading
import random
import time

sem = threading.Semaphore()


load_dotenv()
DATABASE_URL ='mongodb+srv://RobotImprov:'+os.environ.get("password")+'@cluster0.r2dn6t9.mongodb.net/?retryWrites=true&w=majority'
#client = pymongo.MongoClient('mongodb://localhost:27017')
client = pymongo.MongoClient(DATABASE_URL)
db = client.RobotImprov
ans = db.Answers
sessions = db.Sessions

app = Flask(__name__)

links = [
            {   'id'    : 1,
                'link'  : 'https://www.youtube.com/embed/GukLDtotnS8?enablejsapi=1'},
            {   'id'    : 2,
                'link'  : 'https://www.youtube.com/embed/HP4YJ7Rjdn4?enablejsapi=1'},
            {   'id'    : 3,
                'link'  : 'https://www.youtube.com/embed/XSNpV-SRHU8?enablejsapi=1'},
            {   'id'    : 4,
                'link'  : 'https://www.youtube.com/embed/FK-HFeeOLn8?enablejsapi=1'},
          ]

@app.route("/")
def index():
    return render_template('homepage.html')

@app.route('/start', methods=['POST'])
def start():
        
    sem.acquire()
    video = random.choice(links)

    id = 0
    id_available = False

    while id_available == False:
        session_query = {'id': id}
        session = sessions.find_one(session_query)
        print(session)
        if session == None:
            id_available = True
        else:
            id = id + 1

    video_ids = []
    video_ids.append(video['id'])

    now = time.localtime()
    now = int(time.strftime("%Y%m%d%H%M", now))

    cached_session = {}
    cached_session['id'] = id
    cached_session['video_ids'] = video_ids
    cached_session['creation_time'] = now

    clean_inactive_sessions()
    sessions.insert_one(cached_session)
    sem.release()
    return render_template('evaluation.html', link = video['link'], id = id)


@app.route('/handle_answer', methods=['POST'])
def handle_answer():
    session_id = int(request.form.get("session_id"))
    print("id: " + str(session_id))

    session_query = {'id': session_id}
    session = sessions.find_one(session_query)

    answer = request.form['answerinput']
    answer = json.loads(answer)
    answer['video_id'] = session['video_ids'][-1]
    
    print("here is the dict: " )
    print(answer)

    print("here is the likes: " + str(answer["like"]))
    print("here is the dislikes: " + str(answer["dislike"]))
    ans.insert_one(answer)
    new_video_block = 'display: block'
    if len(session['video_ids']) == 4:
        new_video_block = 'display: none'
    return render_template('thanks.html', id = session['id'], hide_next_btn = new_video_block)

@app.route('/handle_another_video', methods=['POST'])
def handle_another_video():
    session_id = int(request.form.get("session_id"))
    print("id: " + str(session_id))

    session_query = {'id': session_id}
    session = sessions.find_one(session_query)

    seen_videos = session['video_ids']
    videos_to_see = []
    for video in links:
        if video['id'] not in seen_videos:
            videos_to_see.append(video)
    video = random.choice(videos_to_see)

    seen_videos.append(video['id'])
    newvalues = {"$set": {"video_ids": seen_videos}}
    sessions.update_one(session_query, newvalues)

    return render_template('evaluation.html', link = video['link'], id = session['id'])

def clean_inactive_sessions():
  now = time.localtime()
  now = int(time.strftime("%Y%m%d%H%M", now))
  sessions.delete_many({'creation_time' : { "$lt" : now - 10}})


if __name__ == '__main__':
    app.run(debug=True)
    
