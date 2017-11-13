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
        var gpsHub = new signalR.HubConnection(ApiUrl.replace('api/', '') + 'gps');
        var sonarHub = new signalR.HubConnection(ApiUrl.replace('api/', '') + 'sonar');

        gpsHub.on('gpsUpdate', function (gpsData) {
            var coords = {lat: gpsData.latitude, lng: gpsData.longitude};

            vm.gpsData = gpsData;
            vm.willyMarker.lat = coords.lat;
            vm.willyMarker.lng = coords.lng;
            vm.path.push(coords);

            if (vm.historyMarkers.length > 49)
                vm.historyMarkers.shift();
            if (vm.path.length > 49)
                vm.path.shift();
            console.log(vm.path);
            $scope.$apply();
        });
        sonarHub.on('sonarUpdate', function (sonarData) {
            vm.sonarData = sonarData;
            $scope.$apply();
        });

        gpsHub.start();
        sonarHub.start();

        vm.path = [];
        vm.historyMarkers = [];

        vm.watchOptions = {
            paths: {
                individual: {type: 'watch'}, //this keeps infdigest errors from happening.... (deep by default)
                type: 'watchCollection'
            }
        };
        vm.windesheim = {
            lat: 52.499937,
            lng: 6.0801503,
            zoom: 17,
            message: 'windesheim'
        };
        vm.willyMarker = {
            lat: 52.499937,
            lng: 6.0801503,
            icon: {
                iconUrl: 'images/willy_topdown.png',
                iconSize:     [35, 45], // size of the icon
                iconAnchor:   [17.5, 22.5], // point of the icon which will correspond to marker's location
            }
        };
        vm.willyPaths = {
            p1: {
                color: '#2196F3',
                weight: 5,
                latlngs: vm.path,
                label: {message: "<h3>Route from Vienna to Paris</h3><p>Distance: 1211km</p>"}
            }
        };
        vm.markers = {
            mainMarker: vm.willyMarker
        };

        // Stop both hubs when the scope is destroyed (when the user navigates away)
        $scope.$on('$destroy', function () {
            gpsHub.stop();
            sonarHub.stop();
        });
    }

})();
