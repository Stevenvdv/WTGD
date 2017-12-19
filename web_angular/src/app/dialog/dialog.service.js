(function () {
    'use strict';
    angular.module('app').service('dialogService', function ($uibModal) {
        this.confirm = function (message, onConfirm, onDecline) {
            // Show the command creation modal
            var modalInstance = $uibModal.open({
                templateUrl: 'app/dialog/dialog.confirmModal.view.html',
                controller: 'confirmModalCtrl',
                controllerAs: 'vm',
                resolve: {
                    message: function () {
                        return message;
                    }
                }
            });
            // Save the command
            modalInstance.result.then(function (result) {
                if (result === 'confirm') {
                    onConfirm();
                }
            },function () {
                if (onDecline) {
                    onDecline();
                }
            });
        }
    });
})();
