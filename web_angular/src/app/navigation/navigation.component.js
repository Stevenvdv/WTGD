(function () {
    'use strict';

    angular.module('app').component('navigation', {
        controller: NavigationController,
        controllerAs: 'vm',
        templateUrl: 'app/navigation/navigation.view.html'
    });

    /** @ngInject */
    function NavigationController($scope, ApiUrl) {
        var vm = this;

        // Setup SignalR
        var connection = new signalR.HubConnection(ApiUrl.replace('api/', '') + 'gps');
        connection.on('gpsUpdate', function (gpsData) {
            vm.gpsData = gpsData;
            vm.marker.coords.latitude = gpsData.latitude;
            vm.marker.coords.longitude = gpsData.longitude;
            $scope.$apply();
            console.log(gpsData);
        });
        connection.start();

        // Spawn the map, default the position to Zwolle's city center
        vm.map = { center: { latitude: 52.513, longitude: 6.095 }, zoom: 16 };
        // Create a marker used to display Willy's position
        vm.marker = {
            id: 0,
            coords: {
                latitude: 52.513,
                longitude: 6.095
            }
        };
    }

})();
