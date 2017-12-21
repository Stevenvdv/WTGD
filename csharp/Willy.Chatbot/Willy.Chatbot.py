# -*- coding: utf-8 -*-
from chatterbot import ChatBot
from chatterbot.trainers import ChatterBotCorpusTrainer
from flask import Flask, jsonify, request, abort
import uuid

app = Flask(__name__)
bots = {}

def add_bot(guid):
    bots[guid] = ChatBot(
        "Willy de afvalrobot",
        storage_adapter="chatterbot.storage.SQLStorageAdapter",
	    database='./database.sqlite3',
        logic_adapters=[	
            "chatterbot.logic.BestMatch"
        ],
        read_only=True
    )
    bots[guid].set_trainer(ChatterBotCorpusTrainer)

@app.route("/")
def home():
    return jsonify(bots.keys())

@app.route("/create")
def create():
    guid = str(uuid.uuid4())
    add_bot(guid)
    return guid

@app.route("/remove")
def remove():
    guid = request.args.get('guid')
    if guid in bots:
        bots.pop(guid);
        return ('', 204)
    else:
        abort(404)

@app.route("/message")
def get_bot_response():
    guid = request.args.get('guid')
    message = request.args.get('msg')
    if guid in bots:
        return str(bots[guid].get_response(message))
    else:
        abort(404)

@app.route("/train")
def train():
    bot = ChatBot(
        "Willy de afvalrobot",
        storage_adapter="chatterbot.storage.SQLStorageAdapter",
	    database='./database.sqlite3',
        logic_adapters=[	
            "chatterbot.logic.BestMatch"
        ]
    )
    bot.set_trainer(ChatterBotCorpusTrainer)
    bot.train(
            "./data/corpus/dutch_willy/botprofile.yml",
            "./data/corpus/dutch_willy/conversations.yml",
            "./data/corpus/dutch_willy/environment.yml",
            "./data/corpus/dutch_willy/facts.yml",
            "./data/corpus/dutch_willy/food.yml",
            "./data/corpus/dutch_willy/greetings.yml",
            #"./data/corpus/dutch_willy/humor.yml",
            #"./data/corpus/dutch_willy/sience.yml",
    )
    return reset()

@app.route("/reset")
def reset():
    bots.clear()
    return ('', 204)

if __name__ == "__main__":
    app.run()