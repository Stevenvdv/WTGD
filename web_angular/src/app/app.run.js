(function () {
    'use strict';

    angular.module('app').run(runBlock);

    /** @ngInject */
    function runBlock($log, $rootScope, $transitions) {
        // Set the page title on route change
        $transitions.onSuccess({}, function ($transition) {
            $rootScope.title = $transition.to().title;
        });

        $log.debug('App run block end');
    }

})();
