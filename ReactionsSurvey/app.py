from flask import Flask, render_template, request, url_for, flash, redirect, session
import pymongo, json
import os
from dotenv import load_dotenv
from jinja2 import Environment
import copy, random
import time
import threading

sem = threading.Semaphore()


#cached_answers = {}

debug_links = [
    {'reaction'             : "attack",
     'link'                 : "https://www.youtube.com/embed/fW9Tq6WwCL4?controls=1&autoplay=0&playlist=fW9Tq6WwCL4&loop=1&vq=hd1080"},
    {'reaction'             : "scolding",             
     'link'                 : "https://www.youtube.com/embed/kg3jIsZ1hrM?controls=1&autoplay=0&playlist=kg3jIsZ1hrM&loop=1&vq=hd1080"}
]

reaction_links = [
    {'reaction'             : "attack",
     'link'                 : "https://www.youtube.com/embed/fW9Tq6WwCL4?controls=1&autoplay=0&playlist=fW9Tq6WwCL4&loop=1&vq=hd1080"},
    {'reaction'             : "scolding",             
     'link'                 : "https://www.youtube.com/embed/kg3jIsZ1hrM?controls=1&autoplay=0&playlist=kg3jIsZ1hrM&loop=1&vq=hd1080"},
    {'reaction'             : "intimidate",          
      'link'                : "https://www.youtube.com/embed/flDDhqZuv_E?controls=1&autoplay=0&playlist=flDDhqZuv_E&loop=1&vq=hd1080"},
    {'reaction'             : "grudge",              
      'link'                : "https://www.youtube.com/embed/E9d__dZdeHo?controls=1&autoplay=0&playlist=E9d__dZdeHo&loop=1&vq=hd1080"},
    {'reaction'             : "sharing_happiness",    
     'link'                 : "https://www.youtube.com/embed/LYbnfvBuH08?controls=1&autoplay=0&playlist=LYbnfvBuH08&loop=1&vq=hd1080"},
    {'reaction'             : "happy_person",        
      'link'                : "https://www.youtube.com/embed/UPpqHDG5coU?controls=1&autoplay=0&playlist=UPpqHDG5coU&loop=1&vq=hd1080"},
    {'reaction'             : "satisfaction",         
     'link'                 : "https://www.youtube.com/embed/SKueU78V_Yc?controls=1&autoplay=0&playlist=SKueU78V_Yc&loop=1&vq=hd1080"},
    {'reaction'             : "sharing_fear",         
     'link'                 : "https://www.youtube.com/embed/PhfO0okb2kQ?controls=1&autoplay=0&playlist=PhfO0okb2kQ&loop=1&vq=hd1080"},
    {'reaction'             : "running_away",         
     'link'                 : "https://www.youtube.com/embed/vpaJ5MPX06w?controls=1&autoplay=0&playlist=vpaJ5MPX06w&loop=1&vq=hd1080"},
    {'reaction'             : "sharing_sadness",     
      'link'                : "https://www.youtube.com/embed/LDUldVbcoPM?controls=1&autoplay=0&playlist=LDUldVbcoPM&loop=1&vq=hd1080"},
    {'reaction'             : "disappointment",      
      'link'                : "https://www.youtube.com/embed/axnxijdAHJQ?controls=1&autoplay=0&playlist=axnxijdAHJQ&loop=1&vq=hd1080"},
    {'reaction'             : "surprise",             
     'link'                 : "https://www.youtube.com/embed/XS8aKDBrVGg?controls=1&autoplay=0&playlist=XS8aKDBrVGg&loop=1&vq=hd1080"},
    {'reaction'             : "disbelief",           
      'link'                : "https://www.youtube.com/embed/5shT9QnaIcE?controls=1&autoplay=0&playlist=5shT9QnaIcE&loop=1&vq=hd1080"},
    {'reaction'             : "astonishment",        
      'link'                : "https://www.youtube.com/embed/ojnUWut8n8c?controls=1&autoplay=0&playlist=ojnUWut8n8c&loop=1&vq=hd1080"}
]

id = 0

is_debug = False

if is_debug == True:
    links = debug_links
else:
    links = reaction_links

load_dotenv()
DATABASE_URL ='mongodb+srv://RobotImprov:'+os.environ.get("password")+'@cluster0.r2dn6t9.mongodb.net/?retryWrites=true&w=majority'
#client = pymongo.MongoClient('mongodb://localhost:27017')
client = pymongo.MongoClient(DATABASE_URL)
db = client.ReactionsSurvey
ans = db.Answers
sessions = db.Sessions

app = Flask(__name__)

@app.route("/")
def index():
    return render_template('homepage.html')

@app.route('/start', methods=['POST'])
def start():
  
  sem.acquire()
  link = []
  link.append(random.choice(links))

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


  #global cached_answers
  empty_answer = {"attack"            : None,
        "scolding"          : None,
        "intimidate"        : None,
        "grudge"            : None,
        "sharing_happiness" : None,
        "happy_person"      : None,
        "satisfaction"      : None,
        "sharing_fear"      : None,
        "running_away"      : None,
        "sharing_sadness"   : None,
        "disappointment"    : None,
        "surprise"          : None,
        "disbelief"         : None,
        "astonishment"      : None}

  now = time.localtime()
  now = int(time.strftime("%Y%m%d%H%M", now))

  cached_session = {}
  cached_session['id'] = id
  cached_session['answer'] = empty_answer
  cached_session['creation_time'] = now

  clean_inactive_sessions()
  sessions.insert_one(cached_session)
  sem.release()
  return render_template('evaluation.html', links = link, id = id)


@app.route('/handle_answer', methods=['POST'])
def handle_answer():
  session_id = int(request.form.get("session_id"))
  print("id: " + str(session_id))

  session_query = {'id': session_id}
  session = sessions.find_one(session_query)
  

  for item in links:
      input = request.form.get(item['reaction'])
      if not input== None: 
        reaction, guess = input.split('-')
        session['answer'][reaction] = guess
        sessions.update_one({'id': session_id},{"$set": {"answer" : session['answer']}})
        #cached_answers[session_id]['answer'][reaction] = guess

  print(session)

  if None in session['answer'].values():
    ans_to_give = {}
    for key,value in session['answer'].items():
       if value == None:
          ans_to_give[key] = value
    rnd_key = random.choice(list(ans_to_give.keys()))
    link = []
    link.append(getLinkByReaction(rnd_key))
    return render_template('evaluation.html', links = link, id = session_id)
  else:
    ans.insert_one(session['answer'])
    sessions.delete_one({'id': session_id})
    return render_template('thanks.html')


def getLinkByReaction(reaction):
   for item in links:
      if item['reaction'] == reaction:
         return item
      
def clean_inactive_sessions():
  now = time.localtime()
  now = int(time.strftime("%Y%m%d%H%M", now))
  sessions.delete_many({'creation_time' : { "$lt" : now - 10}})

