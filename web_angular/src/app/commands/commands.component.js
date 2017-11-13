(function () {
    'use strict';
    angular.module('app').component('commands', {
        controller: CommandsController,
        controllerAs: 'vm',
        templateUrl: 'app/commands/commands.view.html'
    });

    /** @ngInject */
    function CommandsController($scope, ApiUrl) {
        var vm = this;
        vm.commandLines = [];
        // Setup SignalR
        var commandHub = new signalR.HubConnection(ApiUrl.replace('api/', '') + 'command');

        commandHub.on('commandOutput', function (commandOutput) {
            vm.commandLines.push(commandOutput);
            if (vm.commandLines.length > 5000)
                vm.commandLines.shift();
            $scope.$apply();

            // After applying the scope, scroll into view
            var scroller = document.getElementById('commands');
            scroller.scrollTop = scroller.scrollHeight;
        });
        commandHub.start();

        vm.runCommand = function (command) {
            commandHub.invoke('ExecuteCommand', command);
        };

        vm.submitCommandInput = function () {
            if (vm.commandInput) {
                vm.runCommand(vm.commandInput);
                vm.commandInput = '';
            }
        };

        // Stop the hub when the scope is destroyed (when the user navigates away)
        $scope.$on('$destroy', function () {
            commandHub.stop();
        });
    }
})();
