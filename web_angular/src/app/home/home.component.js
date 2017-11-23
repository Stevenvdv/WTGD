(function () {
    'use strict';

    angular.module('app').component('home', {
        controller: HomeController,
        controllerAs: 'vm',
        templateUrl: 'app/home/home.view.html'
    });

    /** @ngInject */
    function HomeController($http, ApiUrl) {
        var vm = this;

        vm.getSystemState = function () {
            $http.get(ApiUrl + 'controlpanel/systemstate').then(function (result) {
                console.log(result);
            })
        };
        vm.getSystemState();
    }

})();
