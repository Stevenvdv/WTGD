(function () {
    'use strict';

    angular.module('app').component('navbar', {
        controller: NavbarController,
        controllerAs: 'vm',
        templateUrl: 'app/navbar/navbar.view.html'
    });

    /** @ngInject */
    function NavbarController($rootScope, authenticationService) {
        var vm = this;
        vm.currentUser = authenticationService.authentication.username;

        $rootScope.$on('authorized', function () {
            vm.currentUser = authenticationService.authentication.username;

        });
        $rootScope.$on('deauthorized', function () {
            vm.currentUser = null;
        });

        vm.logOut = function () {
            authenticationService.logOut();
        }
    }

})();
