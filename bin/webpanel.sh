#!/bin/bash
# file: webpanel.sh

cd ~/Documents/willyweb
screen -dmS webpanel dotnet Willy.Web.dll
sleep 2

