(function () {
    'use strict';

    angular.module('app').component('navbar', {
        controller: NavbarController,
        controllerAs: 'vm',
        templateUrl: 'app/navbar/navbar.view.html'
    });

    /** @ngInject */
    function NavbarController() {
    }

})();
