;; Copyright 2026 Intelligent Robotics Lab
;;
;; This file is part of the project EasyFleet
;; Licensed under the Apache License, Version 2.0 (the "License");
;; you may not use this file except in compliance with the License.
;; You may obtain a copy of the License at
;;
;;     http://www.apache.org/licenses/LICENSE-2.0
;;
;; Unless required by applicable law or agreed to in writing, software
;; distributed under the License is distributed on an "AS IS" BASIS,
;; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
;; See the License for the specific language governing permissions and
;; limitations under the License.

;; Warehouse A logistics: scouts (perception) locate packages in the rack
;; bays, carriers (manipulation) pick them and drop them at the docks.
;; A location holds one robot at a time. Each action runs a behavior tree
;; whose leaves call the robot's EasyFleet capabilities.
(define (domain warehouse)
(:requirements :strips :typing :fluents :durative-actions)

(:types
  robot
  location
  package
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (free ?l - location)
  (idle ?r - robot)
  (can_perceive ?r - robot)
  (can_manipulate ?r - robot)
  (hand_empty ?r - robot)
  (is_dock ?l - location)
  (package_at ?p - package ?l - location)
  (located ?p - package)
  (carrying ?r - robot ?p - package)
)

(:functions
  ;; meters along the routes; robots average about 0.4 m/s
  (distance ?from ?to - location)
)

;; navigation capability
(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration (/ (distance ?from ?to) 0.4))
  :condition (and
    (at start (robot_at ?r ?from))
    (at start (free ?to))
    (at start (idle ?r))
  )
  :effect (and
    (at start (not (robot_at ?r ?from)))
    (at start (free ?from))
    (at start (not (free ?to)))
    (at start (not (idle ?r)))
    (at end (robot_at ?r ?to))
    (at end (idle ?r))
  )
)

;; perception capability
(:durative-action inspect
  :parameters (?r - robot ?p - package ?l - location)
  :duration (= ?duration 5)
  :condition (and
    (at start (can_perceive ?r))
    (at start (idle ?r))
    (over all (robot_at ?r ?l))
    (over all (package_at ?p ?l))
  )
  :effect (and
    (at start (not (idle ?r)))
    (at end (located ?p))
    (at end (idle ?r))
  )
)

;; manipulation capability
(:durative-action pick
  :parameters (?r - robot ?p - package ?l - location)
  :duration (= ?duration 10)
  :condition (and
    (at start (can_manipulate ?r))
    (at start (idle ?r))
    (at start (hand_empty ?r))
    (at start (located ?p))
    (at start (package_at ?p ?l))
    (over all (robot_at ?r ?l))
  )
  :effect (and
    (at start (not (idle ?r)))
    (at start (not (hand_empty ?r)))
    (at start (not (package_at ?p ?l)))
    (at end (carrying ?r ?p))
    (at end (idle ?r))
  )
)

;; manipulation capability
(:durative-action drop
  :parameters (?r - robot ?p - package ?l - location)
  :duration (= ?duration 10)
  :condition (and
    (at start (idle ?r))
    (at start (carrying ?r ?p))
    (at start (is_dock ?l))
    (over all (robot_at ?r ?l))
  )
  :effect (and
    (at start (not (idle ?r)))
    (at start (not (carrying ?r ?p)))
    (at end (package_at ?p ?l))
    (at end (hand_empty ?r))
    (at end (idle ?r))
  )
)
)
