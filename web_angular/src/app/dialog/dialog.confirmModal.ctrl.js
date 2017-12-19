(function () {
    'use strict';
    angular.module('app').controller('confirmModalCtrl', function ($uibModalInstance, message) {
        var vm = this;
        vm.message = message;

        vm.confirm = function () {
            $uibModalInstance.close('confirm');
        };
        vm.cancel = function () {
            $uibModalInstance.dismiss('cancel');
        };
    });
})();
