(function () {
    'use strict';

    angular.module('app').component('home', {
        controller: HomeController,
        controllerAs: 'vm',
        templateUrl: 'app/home/home.view.html'
    });

    /** @ngInject */
    function HomeController() {
    }

})();
