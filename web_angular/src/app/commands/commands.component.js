(function () {
    'use strict';
    angular.module('app').component('commands', {
        controller: CommandsController,
        controllerAs: 'vm',
        templateUrl: 'app/commands/commands.view.html'
    });

    /** @ngInject */
    function CommandsController($scope, Command, ApiUrl, $uibModal, Notification, dialogService) {
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

        vm.runCommand = function (command) {
            commandHub.invoke('ExecuteCommand', command);
        };

        vm.getCommands = function () {
            Command.query(function (commands) {
                console.log(commands);
                vm.commands = commands;
            })
        };

        vm.addCommand = function () {
            // Show the command creation modal
            var modalInstance = $uibModal.open({
                templateUrl: 'app/commands/commands.createModal.view.html',
                controller: 'createCommandModalCtrl',
                controllerAs: 'vm'
            });
            // Save the command
            modalInstance.result.then(function (result) {
                result.$save(function () {
                    // Refresh the list of commands
                    vm.getCommands();
                    Notification.success({
                        message: 'Successfully added the <strong>' + result.name + '</strong> command.',
                        title: 'Command creation'
                    });
                });
            });
        };

        vm.deleteCommand = function (command) {
          dialogService.confirm('Are you sure you want to remove this command?', function (result) {
              Notification({
                  message: 'Successfully deleted the <strong>' + command.name + '</strong> command.',
                  title: 'Command deletion'
              });
          });
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

        commandHub.start();
        vm.getCommands();
    }
})();
