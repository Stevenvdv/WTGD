(function () {
    'use strict';
    angular.module('app').component('remoteControl', {
        controller: RemoteControlController,
        controllerAs: 'vm',
        templateUrl: 'app/remoteControl/remoteControl.view.html'
    });

    /** @ngInject */
    function RemoteControlController($scope, $timeout, SignalRUrl, authenticationService) {
        var vm = this;
        var lastX = 0;
        var lastY = 0;

        var options = {
            color: '#444',
            size: 150,
            zone: document.getElementById('joystick-container')
        };
        var manager = nipplejs.create(options);

        // Setup SignalR
        var hubOptions = {
            transport: 0, // use WebSockets, this is the default but since options is overwritten it needs to be supplied
            jwtBearer: function () { // pass the JWT token
                return authenticationService.authentication.token;
            }
        };
        var rcHub = new signalR.HubConnection(SignalRUrl + 'controlpanel/remoteControl', hubOptions);
        rcHub.start();

        manager.on('added', function (evt, nipple) {
            nipple.on('start move end dir plain', function (evt, data) {
                if (evt.type === 'move') {
                    var x = 0, y = 0;
                    // Willy is quite sensitive so turn down the distance by a fair margin, could make this a slider
                    var distance = data.distance / 30;
                    if (((data.angle.degree < 90) && (data.angle.degree > 0)) || ((data.angle.degree < 360) && (data.angle.degree > 270))) {
                        x = Math.cos(data.angle.radian) * distance;
                        y = Math.sin(data.angle.radian) * distance;
                    }
                    if (((data.angle.degree < 180) && (data.angle.degree > 90)) || ((data.angle.degree < 270) && (data.angle.degree > 180))) {
                        x = -Math.cos(Math.PI - data.angle.radian) * distance;
                        y = Math.sin(Math.PI - data.angle.radian) * distance;
                    }
                    // Invert the X and half the sensitivity
                    x = (x * -1) / 2;

                    // Only send new values if they are different enough
                    if (Math.abs(lastX - x) > 0.02 || Math.abs(lastY - y) > 0.02) {
                        rcHub.invoke('Move', x, y);
                        lastX = x;
                        lastY = y;
                    }
                } else if (evt.type === 'end') {
                    lastX = 0;
                    lastY = 0;
                    $timeout(function () {
                        rcHub.invoke('Move', 0.0, 0.0)
                    }, 250);
                    $timeout(function () {
                        rcHub.invoke('Move', 0.0, 0.0)
                    }, 500)

                }
            });
        }).on('removed', function (evt, nipple) {
            nipple.off('start move end dir plain');
        });

        // Stop both hubs when the scope is destroyed (when the user navigates away)
        $scope.$on('$destroy', function () {
            // Clean up nipples?
        });
    }

})();
