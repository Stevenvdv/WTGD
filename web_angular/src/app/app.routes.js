(function () {
    'use strict';

    angular.module('app').config(routerConfig);

    /** @ngInject */
    function routerConfig($stateProvider, $urlRouterProvider) {
        $stateProvider.state('home', {
            url: '/',
            component: 'home',
            title: 'Home'
        }).state('login', {
            url: '/login',
            component: 'login',
            title: 'Login'
        }).state('navigation', {
            url: '/navigation',
            component: 'navigation',
            title: 'Navigation'
        }).state('commands', {
            url: '/commands',
            component: 'commands',
            title: 'Commands'
        });

        $urlRouterProvider.otherwise('/');
    }

})();
