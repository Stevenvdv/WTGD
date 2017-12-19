(function () {
    'use strict';
    angular.module('app').controller('createCommandModalCtrl', function ($uibModalInstance, Command) {
        var vm = this;
        vm.model = new Command();
        vm.save = function () {
            $uibModalInstance.close(vm.model);
        };

        vm.cancel = function () {
            $uibModalInstance.dismiss('cancel');
        };
    });
})();
