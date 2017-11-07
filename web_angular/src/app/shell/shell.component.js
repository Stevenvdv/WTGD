(function () {
    'use strict';
    angular.module('app').component('shell', {
        controller: ShellController,
        controllerAs: 'vm',
        templateUrl: 'app/shell/shell.view.html'
    });

    /** @ngInject */
    function ShellController($scope, ApiUrl) {
        var vm = this;
        vm.shellConnected = false;
        vm.shellLines = [];
        // Setup SignalR
        var shellHub = new signalR.HubConnection(ApiUrl.replace('api/', '') + 'shell');

        shellHub.on('connectResult', function (connectResult) {
            vm.shellConnected = connectResult;
            $scope.$apply();
        });
        shellHub.on('commandResult', function (commandResult) {
            vm.shellLines.push(commandResult);
            if (vm.shellLines.length > 100)
                vm.shellLines.shift();
            $scope.$apply();

            // After applying the scope, scroll into view

                var scroller = document.getElementById('shell');
                scroller.scrollTop = scroller.scrollHeight;
        });
        shellHub.start();

        vm.connect = function (login) {
            if (vm.shellConnected) {
                return;
            }
            console.log(shellHub);
            shellHub.invoke('ConnectToSsh', login.username, login.password);
        };
        vm.runCommand = function (command) {
            if (!vm.shellConnected) {
                return;
            }

            shellHub.invoke('RunCommand', command);
        };

        vm.submitShellInput = function () {
            if (vm.shellInput) {
                vm.runCommand(vm.shellInput);
                vm.shellInput = '';
            }
        };

        // Stop the hub when the scope is destroyed (when the user navigates away)
        $scope.$on('$destroy', function() {
            shellHub.stop();
        });
    }
})();
