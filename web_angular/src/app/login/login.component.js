(function () {
    'use strict';

    angular.module('app').component('login', {
        controller: LoginController,
        controllerAs: 'vm',
        templateUrl: 'app/login/login.view.html'
    });

    /** @ngInject */
    function LoginController(authenticationService, Notification, $state) {
        var vm = this;

        // If already logged in the user has no need to be here
        if (authenticationService.authentication.isAuth) {
            $state.go('home');
        }

        vm.login = function (loginData) {
            authenticationService.login(loginData).then(function () {
                Notification.success({message: 'Welcome, ' + loginData.username + '.', title: 'Successfully logged in'})
                $state.go('home');
            }).catch(function (e) {
                if (e.status === 400) {
                    Notification.error({message: 'Invalid username or password.', title: 'Login failed'})
                } else {
                    Notification.error({message: 'Something went wrong on the server.', title: 'Login failed'});
                    console.log(e);
                }
            });
        }
    }

})();
