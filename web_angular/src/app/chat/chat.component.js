(function () {
    'use strict';
    angular.module('app').component('chat', {
        controller: chatController,
        controllerAs: 'vm',
        templateUrl: 'app/chat/chat.view.html'
    });

    /** @ngInject */
    function chatController($scope, SignalRUrl, authenticationService, hotkeys) {
        var createjs = window.createjs;
        var vm = this;
        vm.chatLines = [];

        // Setup SignalR
        var options = {
            transport: 0, // use WebSockets, this is the default but since options is overwritten it needs to be supplied
            jwtBearer: function () { // pass the JWT token
                return authenticationService.authentication.token;
            }
        };
        var chatHub = new signalR.HubConnection(SignalRUrl + 'social/chat', options);

        // Setup speech recognition
        vm.speechRecognitionAvailable = false;
        vm.speechListening = false;
        vm.speechTalking = false;
        vm.speechFinalResult = '';
        vm.speechInterimResult = '';
        hotkeys.bindTo($scope)
            .add({
                combo: 'space',
                description: 'Start speech recognition',
                callback: function () {
                    vm.startRecognition();
                }
            });

        if ('webkitSpeechRecognition' in window) {
            vm.recognition = new webkitSpeechRecognition();
            vm.recognition.continuous = false;
            vm.recognition.interimResults = true;
            vm.recognition.lang = 'nl-NL';

            vm.recognition.onstart = function () {
                vm.speechListening = true;
                $scope.$apply();
            };
            vm.recognition.onresult = function (event) {
                var final_transcript = '';
                var interim_transcript = '';

                for (var i = event.resultIndex; i < event.results.length; ++i) {
                    if (event.results[i].isFinal) {
                        final_transcript += event.results[i][0].transcript;
                    } else {
                        interim_transcript += event.results[i][0].transcript;
                    }
                }

                vm.speechFinalResult = final_transcript;
                vm.speechInterimResult = interim_transcript;
                vm.chatInput = interim_transcript;
                $scope.$apply();
            };
            vm.recognition.onerror = function (event) {
                vm.speechListening = false;
                $scope.$apply();
            };
            vm.recognition.onend = function () {
                vm.speechListening = false;
                vm.getResponse(vm.speechFinalResult);

                $scope.$apply();
            };

            vm.speechRecognitionAvailable = true;
        }

        vm.startRecognition = function () {
            vm.recognition.start();
        };

        vm.sayAsWilly = function (sentence) {
            if (vm.speechTalking) {
                return;
            }
            var utterance = new SpeechSynthesisUtterance(sentence);
            utterance.rate = 0.9;
            utterance.pitch = 0.4;
            utterance.lang = 'nl-NL';

            utterance.onstart = function () {
                vm.speechTalking = true;
                $scope.$apply();
            };
            utterance.onend = function () {
                vm.speechTalking = false;
                $scope.$apply();
            };
            utterance.onerror = function () {
                vm.speechTalking = false;
                $scope.$apply();
            };
            window.speechSynthesis.speak(utterance);
        };

        chatHub.on('response', function (chatOutput) {
            vm.addChatLine(' Willy: ' + chatOutput);
            vm.sayAsWilly(chatOutput);
            $scope.$apply();
        });

        vm.getResponse = function (message) {
            if (message && message !== '' && message !== ' ') {
                vm.chatInput = '';
                vm.addChatLine('Jij: ' + message);
                chatHub.invoke('GetResponse', message);
            }
        };

        vm.submitChatInput = function () {
            if (vm.chatInput) {
                vm.getResponse(vm.chatInput);
                vm.chatInput = '';
            }
        };

        vm.addChatLine = function (line) {
            vm.chatLines.push(line);
            if (vm.chatLines.length > 5000)
                vm.chatLines.shift();

            // After applying the scope, scroll into view
            var scroller = document.getElementById('chat');
            scroller.scrollTop = scroller.scrollHeight;
        };

        // Stop the hub when the scope is destroyed (when the user navigates away)
        $scope.$on('$destroy', function () {
            chatHub.stop();
        });

        chatHub.start();


        // Avatar
        vm.showAvatar = false;
        vm.toggleShowAvatar = function () {
            if (vm.showAvatar) {
                vm.showAvatar = false;
                return;
            }

            vm.showAvatar = true;
            vm.stage = new createjs.Stage('avatar-canvas');
            //Create a Shape DisplayObject.
            var body = new createjs.Bitmap("images/avatar-body.png");
            var mouth = new createjs.Bitmap("images/avatar-mouth.png");

            vm.stage.addChild(body);
            vm.stage.addChild(mouth);
            //Update stage will render next frame
            vm.stage.update();

            //Update stage will render next frame
            createjs.Ticker.addEventListener("tick", handleTick);

            function handleTick() {

                vm.stage.update();
            }
        }
    }
})();
