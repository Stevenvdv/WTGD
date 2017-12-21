(function () {
    'use strict';
    angular.module('app').component('chat', {
        controller: chatController,
        controllerAs: 'vm',
        templateUrl: 'app/chat/chat.view.html'
    });

    /** @ngInject */
    function chatController($scope, SignalRUrl, authenticationService) {
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

        chatHub.on('response', function (chatOutput) {
            vm.chatLines.push(chatOutput);
            if (vm.chatLines.length > 5000)
                vm.chatLines.shift();
            $scope.$apply();

            // After applying the scope, scroll into view
            var scroller = document.getElementById('chats');
            scroller.scrollTop = scroller.scrollHeight;
        });

        vm.getResponse = function (message) {
            chatHub.invoke('GetResponse', message);
        };

        vm.submitChatInput = function () {
            if (vm.chatInput) {
                vm.getResponse(vm.chatInput);
                vm.chatInput = '';
            }
        };

        // Stop the hub when the scope is destroyed (when the user navigates away)
        $scope.$on('$destroy', function () {
            chatHub.stop();
        });

        chatHub.start();
    }
})();
