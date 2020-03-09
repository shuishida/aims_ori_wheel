^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mongodb_store
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2019-11-11)
------------------
* added python-future to package.xml, which got lost in previous commit for some reasons ...
* corrected typo
* removed flags for most of files for simplicity, also included future.utils, may introduce undesireable dependency though
* resolved python2 backward compatibility issues
  not thouroughly checked though
  Added python version checks, where needed.
  Changes not adjusted to version:
  - print always with brackets
  - excepti XXX, e ==> except XXX as e
  as there is no problem with python2 to best of my knowledge
* adjusted python funtions for python3
* add dependency to package xml
* back to system mongo
* Contributors: Ferenc Balint-Benczedi, Nick Hawes, Shingo Kitagawa, Volker Gabler, vgab

0.5.1 (2019-06-28)
------------------
* added python-future to package.xml
* removed flags for most of files for simplicity, also included future.utils, may introduce undesireable dependency though
* resolved python2 backward compatibility issues
  not thouroughly checked though
  Added python version checks, where needed.
  Changes not adjusted to version:
  - print always with brackets
  - excepti XXX, e ==> except XXX as e
  as there is no problem with python2 to best of my knowledge
* adjusted python funtions for python3
* Contributors: Shingo Kitagawa, Volker Gabler

0.5.0 (2018-12-20)
------------------
* Merge pull request `#231 <https://github.com/strands-project/mongodb_store/issues/231>`_ from bbferka/melodic-devel
  Melodic devel
* add dependency to package xml
* make it run
* back to system mongo
* Contributors: Ferenc Balint-Benczedi, Nick Hawes

0.4.5 (2019-06-28)
------------------

* Allow to bind mongod to host
  The existing solution bind the mongod server to all interfaces. This
  exposes the server to the outside. This could be a potensial security
  issue if someone has access to your network. This change allows you
  to only bind the server to the host specified by the
  `mongodb_host` parameter. The solution was created with a flag
  defaulting to false to prevent this from being a breaking change.
  Specifying the argument `bind_to_host` will add a bind argument to the
  executable:
  ```
  [..., "--bind_ip", self._mongo_host]
  ```
  Info: The `bind_ip` argument [supports hostnames, ipaddresses
  and socket
  paths](https://docs.mongodb.com/manual/reference/program/mongod/#cmdoption-mongod-bind-ip)
  so using `localhost` or `127.0.0.1` is valid for binding to
  localhost only. To bind to all interfaces use `0.0.0.0`.
* adds service to reset all local overrides.
  Exposes a new service `/config_manager/reset_params std_srvs/Trigger`
  that will reset all local overrides their default value. This provides
  a ROS compatible way of getting back to the default values.
* Contributors: Jørgen Borgesen, Nick Hawes, Marc Hanheide

0.4.3 (2019-04-04)
------------------
* Merge pull request `#243 <https://github.com/strands-project/mongodb_store/issues/243>`_ from furushchev/get-publisher
  add method to get num subscribers for insertion
* Merge pull request `#242 <https://github.com/strands-project/mongodb_store/issues/242>`_ from furushchev/shutdown
  shutdown publisher on destruction
* Merge pull request `#241 <https://github.com/strands-project/mongodb_store/issues/241>`_ from furushchev/add-published-at
  append _meta.published_at for stamped messages
* Merge pull request `#240 <https://github.com/strands-project/mongodb_store/issues/240>`_ from furushchev/fix-replicate
  display more info on replication
* add method to get num subscribers for insertion
* shutdown publisher on destruction
* append _meta.published_at for stamped messages
* display more info on replication
* Merge pull request `#234 <https://github.com/strands-project/mongodb_store/issues/234>`_ from jorgenfb/fix/prevent-null-values
  Make sure None / null values reaches the parameter server
* Make sure None / null values reaches the parameter server
* Contributors: Jørgen Borgesen, Nick Hawes, Yuki Furuta

0.4.2 (2018-06-04)
------------------
* instead of lsb-release read /etc/os-version from CMake to find OS version
* Contributors: Ferenc Balint-Benczedi

0.4.1 (2018-05-29)
------------------
* check if env. var. exists if not decide based on OS version
* Contributors: Ferenc Balint-Benczedi

0.4.0 (2018-05-23)
------------------

* if compiler supports it use C++11
* Contributors: Ferenc Balint-Benczedi

0.3.8 (2018-05-02)
------------------
* Merge pull request `#220 <https://github.com/strands-project/mongodb_store/issues/220>`_ from furushchev/replication-with-query
  Replication with query
* mongodb_store: support filter by query on moving entries
* Merge pull request `#218 <https://github.com/strands-project/mongodb_store/issues/218>`_ from furushchev/test-replicator
  Test replication functionality
* mongodb_store: avoid to use rosrun in test code
* mongodb_store: test replication
* mongodb_server: minor bugfix
* Merge pull request `#213 <https://github.com/strands-project/mongodb_store/issues/213>`_ from furushchev/fix/encode-utf8
  mongodb_store: encode utf-8 on filling rosmsg
* Merge remote-tracking branch 'origin/kinetic-devel' into fix/encode-utf8
* Import connection failure
* Merge pull request `#214 <https://github.com/strands-project/mongodb_store/issues/214>`_ from furushchev/option-query-extra
  mongodb_store: message_store_node.py: deprecate using extra servers
* Merge remote-tracking branch 'origin/kinetic-devel' into option-query-extra
  Conflicts:
  mongodb_store/scripts/message_store_node.py
* Merge pull request `#215 <https://github.com/strands-project/mongodb_store/issues/215>`_ from furushchev/remove-dup-projection
  mongodb_store: Remove duplicated codes for projection
* mongodb_store: use launch for test
* mongodb_store: remove duplicated codes for querying with projection
* mongodb_store: message_store_node.py: deprecate using extra servers
* mongodb_store: encode utf-8 on filling rosmsg
* Merge pull request `#212 <https://github.com/strands-project/mongodb_store/issues/212>`_ from furushchev/fix-query-extra
  fix error on querying extra servers
* fix error on querying extra servers
* Contributors: Furushchev, Nick Hawes, Yuki Furuta

0.3.7 (2017-12-14)
------------------
* Merge pull request `#207 <https://github.com/strands-project/mongodb_store/issues/207>`_ from furushchev/fix-mongocxx
  Use system mongocxx client if possible
* Merge pull request `#205 <https://github.com/strands-project/mongodb_store/issues/205>`_ from furushchev/pymongo3
  Support Pymongo 3
* libmongocxx_ros: try to use system mongocxx lib if possible
* Merge pull request `#206 <https://github.com/strands-project/mongodb_store/issues/206>`_ from Pikrass/update-rosparam
  Store the updated value to rosparam when calling set_param
* Store the updated value to rosparam when calling set_param
* mongodb_store.util: use rospy.log functions instead of print
* config_manager.py: fix error on exit
* support pymongo 3.X
* Contributors: Bastien Scher, Nick Hawes, Yuki Furuta

0.3.6 (2017-09-14)
------------------

0.3.5 (2017-09-03)
------------------
* Merge pull request `#203 <https://github.com/strands-project/mongodb_store/issues/203>`_ from strands-project/marc-hanheide-patch-3
  added `topic_tools`as `CATKIN_DEPEND`
* topic_tools as run_depend
* added `topic_tools`as `CATKIN_DEPEND`
* Contributors: Marc Hanheide

0.3.4 (2017-09-03)
------------------

0.3.3 (2017-09-02)
------------------
* Merge pull request `#200 <https://github.com/strands-project/mongodb_store/issues/200>`_ from strands-project/shapeshifter
  mongodb_store: support shapeshifter
* Fixed missing space.
* mongodb_store: support shapeshifter
* Merge pull request `#195 <https://github.com/strands-project/mongodb_store/issues/195>`_ from mbeneto/kinetic-devel
  Added keep_trash parameter
* Added keep_trash parameter
  This parameter enables the user to select to store the items deleted into the trash (default) or not.
* Contributors: Furushchev, Nick Hawes, mbeneto

0.3.2 (2017-08-31)
------------------

0.3.1 (2017-08-24)
------------------
* remove mongoclient libs from CMakeLists
* Contributors: Bruno Lacerda


0.3.0 (2017-08-24)
------------------

0.2.2 (2017-06-28)
------------------

0.2.1 (2017-06-28)
------------------

0.2.0 (2017-06-28)
------------------
* dependencies fixing
* some attempts to build libmongocxx internally
* indigo-0.1.30
* updated Changelogs
* [package.xml] Add link to devel repository
  Without this, from [the package's wiki page](http://wiki.ros.org/mongodb_store) there's no way to tell where the repo is.
* fixing error with launching mongodb_store
* indigo-0.1.29
* updated Changelogs
* Support local datacentre timeout
  The timeout for the datacentre was hardcoded to 10 seconds. However, in
  some environments, for example in a cloud setup, this may not be enough.
  Make this configurable and just default to the previous 10 seconds.
* Merge pull request `#188 <https://github.com/strands-project/mongodb_store/issues/188>`_ from hkaraoguz/projectionerrorfix
  Projectionerrorfix
* Fixed updateClient assignment on copy.
* The projection tests are changed
* Revert back to original mongodb cmd
* [mongodb_store/stests/message_store_cpp_test.cpp] fix test for non-wait insert
* Fixed projection error happening when exclude and include field directives are mixed
* [mongodb_store] add test for non-wait insert
* [mongodb_store] add non-wait insert functionality
* indigo-0.1.28
* updated Changelogs
* Mongo C++ header location not exposed.
  Implemented fix from @ronwalf closes `#176 <https://github.com/strands-project/mongodb_store/issues/176>`_
* Fixed missing return value
* Fixing the compatibility issues of messagestore cpp client with old SOMA versions
* Fixed issue with projection query including fields instead of excluding
* indigo-0.1.27
* updated Changelogs
* [mongodb_store/scripts/mongodb_server.py] connect with localhost when shutdown server
* Fixed if statement
* geotype of ROI has been added
* The geospatial indexing of SOMA ROI objects is added
* Contributors: Ferdian Jovan, Hakan, Isaac I.Y. Saito, Justin Huang, Marc Hanheide, Nick Hawes, Tim Niemueller, Yuki Furuta

0.1.30 (2017-06-23)
-------------------
* [package.xml] Add link to devel repository
  Without this, from [the package's wiki page](http://wiki.ros.org/mongodb_store) there's no way to tell where the repo is.
* fixing error with launching mongodb_store
* Contributors: Ferdian Jovan, Hakan, Isaac I.Y. Saito, Justin Huang

0.1.29 (2017-06-19)
-------------------
* Support local datacentre timeout
  The timeout for the datacentre was hardcoded to 10 seconds. However, in
  some environments, for example in a cloud setup, this may not be enough.
  Make this configurable and just default to the previous 10 seconds.
* Fixed updateClient assignment on copy.
* The projection tests are changed
* Revert back to original mongodb cmd
* [mongodb_store/stests/message_store_cpp_test.cpp] fix test for non-wait insert
* Fixed projection error happening when exclude and include field directives are mixed
* [mongodb_store] add test for non-wait insert
* [mongodb_store] add non-wait insert functionality
* Mongo C++ header location not exposed.
  Implemented fix from @ronwalf closes `#176 <https://github.com/strands-project/mongodb_store/issues/176>`_
* Fixed missing return value
* Fixing the compatibility issues of messagestore cpp client with old SOMA versions
* Fixed issue with projection query including fields instead of excluding
* [mongodb_store/scripts/mongodb_server.py] connect with localhost when shutdown server
* Fixed if statement
* geotype of ROI has been added
* The geospatial indexing of SOMA ROI objects is added
* Contributors: Hakan, Justin Huang, Nick Hawes, Tim Niemueller, Yuki Furuta


0.1.28 (2016-11-09)
-------------------
* Mongo C++ header location now exposed.
* Fixed missing return value
* Fixing the compatibility issues of messagestore cpp client with old SOMA versions
* Fixed issue with projection query including fields instead of excluding
* [mongodb_store/scripts/mongodb_server.py] connect with localhost when shutdown server
* geotype of ROI has been added
* The geospatial indexing of SOMA ROI objects is added
* Contributors: Hakan, Nick Hawes, Yuki Furuta


0.1.27 (2016-11-01)
-------------------
* Fixed if statement
* geotype of ROI has been added
* The geospatial indexing of SOMA ROI objects is added
* Contributors: Hakan

0.1.26 (2016-10-14)
-------------------
* Fixed a bug during soma msg_type check.
* Unit tests have been added for projection.
* Created a new service for querying with projections
* Added checks for new soma object message type. Performed code cleanup
* Projection field to the queries is added
* indigo-0.1.25
* Updating changelogs
* checking object type and adding soma2 fields based on that has been added
* indigo-0.1.24
* updated Changelogs
* Contributors: Hakan, Nick Hawes, hkaraoguz

0.1.25 (2016-04-28)
-------------------
* checking object type and adding soma2 fields based on that has been added
* Contributors: hkaraoguz

0.1.24 (2016-04-19)
-------------------

0.1.23 (2016-04-19)
-------------------
* Using remote mongodb without ros option has been added
* Sort query functionality have been added to cpp interface
* Removal of unnecessary code.
* Update message update method for using timestamp info
* Remove unnecessary print statements
* Fix comments and remove unnecessary print statements
* Modifications in object insertion
* using ROS_HOSTNAME instead of HOSTNAME
  fixes `#160 <https://github.com/strands-project/mongodb_store/issues/160>`_
* GeoSpatial indexing is added for SOMA2 objects and rois
* indigo-0.1.22
* updated Changelogs
* Modifications for geospatial indexing
* Contributors: Marc Hanheide, Nick Hawes, hkaraoguz


0.1.22 (2016-02-23)
-------------------
* Update README.md
  `datacentre.launch` has not existed for a long while, I think it should be `mongodb_store.launch` instead?
* Contributors: Nils Bore


0.1.20 (2015-11-11)
-------------------
* Added some extra robustness to mongodb_play. This means latch does not need to be defined as in `#123 <https://github.com/strands-project/mongodb_store/issues/123>`_.
* Added corrected wait pattern to replicator node start-up
* making mongod using smaller files
  consuming less space. The journals are massive, to the point they can't be created on jenkins.
* Contributors: Marc Hanheide, Nick Hawes



0.1.19 (2015-10-28)
-------------------

0.1.18 (2015-10-28)
-------------------
* Split mongodb_store launch file into two.
  This now provides mongodb_store_inc.launch which assumes that a machine tag has been previously set, and is provided by the machine arg. The original mongodb_store.launch file defines a machine tag then calls the _inc.launch file. This design minimises duplication as far as possible, but is still a bit inelegant. The reason we couldn't do everything with a single file, as discussed in `#148 <https://github.com/strands-project/mongodb_store/issues/148>`_, is we can't test whether an argument has been set in roslaunch so we don't know when to define a machine tag ourselves. The additional boolean flag to dictate this definition was not a nice solution either.
  This supercedes  `#148 <https://github.com/strands-project/mongodb_store/issues/148>`_
* Trying to find why cpp test fails. It appears to be a local issue with library paths not getting passed properly. SOme mention of this is here https://github.com/mikepurvis/ros-install-osx/issues/12
* [mongodb_store] add limit argument for query
* [replicator_node.py] add cancel handler for replication
* [mongodb_store/message_store.py] suppress infinite error output when shutdown without finding mongodb service
* Fix: remove auto-generated databases in /tmp after a test has been completed that may e.g. fill up the harddisk of a Jenkins server
* minor help edit
* logging messages commented
* aded support for start and end times of playback
* Contributors: Moritz Tenorth, Nick Hawes, Vojtech Novak, Yuki Furuta

0.1.17 (2015-09-01)
-------------------


0.1.16 (2015-08-04)
-------------------
* use False as default value of param 'mongodb_use_daemon'
* add option to use already launched mongod
* Fix exception catch.
* Silence wait_for_service.
  This adds some more helpful output if the messages store services can't be found, but produces no output if they are found within 5 seconds.
* Contributors: Chris Burbridge, Yuki Furuta


0.1.15 (2015-05-10)
-------------------
* Add dump path as parameter.
* Contributors: Rares

0.1.14 (2015-04-27)
-------------------

0.1.13 (2015-04-22)
-------------------

0.1.12 (2015-02-09)
-------------------

0.1.11 (2015-02-09)
-------------------
* add switch option to use machine tag
* Contributors: Furushchev

0.1.10 (2014-11-23)
-------------------
* Replication now has db configurable.
  This fixes `#54 <https://github.com/strands-project/mongodb_store/issues/54>`_.
* Added queue_size for indigo
* Contributors: Nick Hawes

0.1.9 (2014-11-18)
------------------

0.1.8 (2014-11-11)
------------------
* use underscore_separation instead of camelCase
* add launch as replSet option
* Fix `#52 <https://github.com/strands-project/mongodb_store/issues/52>`_
* Contributors: Furushchev, lucasb-eyer

0.1.7 (2014-11-09)
------------------
* final and tested version of loader
* latest version of machine tags in launch file
* Contributors: Jaime Pulido Fentanes

0.1.6 (2014-11-06)
------------------
* Launch file to right place this time.
  It seems like the syntax is doing different things in different CMake files though.
* fixing launch file order
* Merge branch 'hydro-devel' of https://github.com/strands-project/mongodb_store into hydro-devel
  Conflicts:
  mongodb_store/launch/mongodb_store.launch
* replacing env for optenv
* Fixed spacing and author info
* Changing launch file to adjust to new machine tag type
* Changing launch file to adjust to new machine tag type
* Contributors: Jailander, Jaime Pulido, Jaime Pulido Fentanes, Nick Hawes

0.1.5 (2014-11-05)
------------------
* Changing the installed location of launch file.
* Added test_mode to launch file.
* Contributors: Nick Hawes

0.1.4 (2014-10-29)
------------------
* Removed debugging message.
* Fixed inclusion of OpenSSL libraries.
  Note the OpenSSL_LIBRARIES != OPENSSL_LIBRARIES
* Edited find mongo script.
* support backward code compatibility; add test code
* add example to sort query
* add sort option on query
* Contributors: Furushchev, Nick Hawes

0.1.3 (2014-10-21)
------------------
* added mongodb-dev as run depend
  to force inclusion in Debian dependencies
* Contributors: Marc Hanheide

0.1.2 (2014-10-20)
------------------
* removed if statement on MONGO_EXPORT
* Using warehouse_ros approach to including MongoDB.
  Added FindMongoDB for this.
* Looks like linking is necessary
* Removing modern c++ for safety.
* Trying to only expose mongo lib for apple.
* Added geometry_msgs back in
* Contributors: Marc Hanheide, Nick Hawes

0.1.1 (2014-10-17)
------------------
* Merge pull request `#99 <https://github.com/strands-project/mongodb_store/issues/99>`_ from hawesie/hydro-devel
  Added geometry_msgs back in to fix `#98 <https://github.com/strands-project/mongodb_store/issues/98>`_
* Added geometry_msgs back in to fix `#98 <https://github.com/strands-project/mongodb_store/issues/98>`_
* Contributors: Nick Hawes

0.1.0 (2014-10-16)
------------------
* Removing author emails as seems to be done on for other packages.
* Added option to specify database.
* Updated URL and description.
* Fixed pacakge name in test launch file.
* Added boost to dependencies.
  Refactoring of package earlier plus this fixes `#95 <https://github.com/strands-project/mongodb_store/issues/95>`_ (hopefully)
* Added cpp changelog to overall package.
* Moved mongodb_store_cpp_client files into mongodb_store package.
* This adds latched recording and playback to the log and playback nodes.
  This is the final part of the functionality to close `#5 <https://github.com/strands-project/mongodb_store/issues/5>`_
* Looking in to date issue.
* Adding meta information into C++ logging.
* Building up type processing knowledge.
* Adding meta information to C++-logged documents.
* Handlings strings which cannot be treated as UTF-8 as binary.
* Debugging ulimit issue.
* First full working version.
  Topics are played back but this is all at the mercy of rospy.sleep
* All processes with sim time.
* Sim time is now awaited correctly.
* Added basic processes for topic publishing.
* Playback node now publishes simulated time.
* Contributors: Nick Hawes

0.0.5 (2014-10-09)
------------------
* Added install target for launch file.
* Fix maintainer in package.xml
* Update package.xml
* Fixed typo.
* Added absolute paths to libraries to ensure that dependent projects get correct linking.
* Contributors: Chris Burbridge, Marc Hanheide, Nick Hawes

0.0.4 (2014-09-13)
------------------
* added mongod
* Add son_manipulator import
* Added test mode to mongodb_server.py
  This generates a random port to listen on and creates a corresponding dbpath under /tmp. The port is tested to see if it's free before it's used.
  This closes `#77 <https://github.com/strands-project/mongodb_store/issues/77>`_ and `#75 <https://github.com/strands-project/mongodb_store/issues/75>`_.
  * added libssl and libcrypto for ubuntu distros where this is needed due to the static nature of the libmongoclient.a
* Added author email and license.
* Contributors: Chris Burbridge, Marc Hanheide, Nick Hawes

0.0.3 (2014-08-18)
------------------
* Renamed rosparams `datacentre_` to `mongodb_`.
  Fixes `#69 <https://github.com/strands-project/ros_datacentre/issues/69>`_
* More renaming to mongodb_store
* Renamed launch file.
* Renamed ros_datacentre to mongodb_store for to fix `#69 <https://github.com/strands-project/ros_datacentre/issues/69>`_.
* Contributors: Nick Hawes

0.0.2 (2014-08-07)
------------------
* Fixed complilation under Ubuntu.
  * removed use of toTimeT()
  * add_definitions(-std=c++0x) to allow new C++ features
* Fix `#65 <https://github.com/strands-project/mongodb_store/issues/65>`_. Check entry exists before accessing value.
* Dynamically choose MongoDB API
  Use Connection if using an older mongopy, otherwise use MongoClient.
* Remove dependency on bson > 2.3
  Use old hook/default interface to avoid having to install bson 2.3 from
  pip.
* Remove dependency on pymongo > 2.3
  In older versions of pymongo, Connection serves the same purpose as
  MongoClient. Updated scripts to use Connection instead of MongoClient.
  This allows the package to work with the existing rosdep definitions for
  python-pymongo (shich use the .deb version).
* Fix `#63 <https://github.com/strands-project/mongodb_store/issues/63>`_. Update pass through son manipulators.
* Fix `#60 <https://github.com/strands-project/mongodb_store/issues/60>`_. Add SONManipulator for xmlrpclib binary data.
* datacentre documentation for python
* docstrings in util module
* message store docstrings
* ready for update to use google docstrings
* adding processing of source documentation
* sphinx configuration and index
* sphinx framework for documentation
* Waiting for datacentre.
* Merge pull request `#49 <https://github.com/strands-project/mongodb_store/issues/49>`_ from hawesie/hydro-devel
  Added replication for message store
* Changed collections type to StringList to allow for datacentre comms to task schduler.
* Change action definition to use duration into the past.
* Switched default time to 24 hours ago rather than now, to allow easier use in scheduler.
* Finishing off replicator node.
  * Added to launch file
  * Added to README
  * made client time 24 hours
* Added some minor sanity checks.
* Working and tested dump and restore with time bounds.
* Added dump and restore.
* Fixed empty list error.
* Adding Machine tag to datacentre.launch
* Tested replication and it passes first attempts.
* Adding first pass stuff for replication.
* Deletion now actually deletes...
* Added cpp example of logging multiple messages together.
* Added example of way to log related events to message store.
* Added examples of id-based operations.
* Added update_id method for updating stored object using ObjectID.
* Added updated time and caller too.
* Added inserter id and time to meta.
* Made wait more obvious
* Working binary with pointclouds.
* Added cpp example of logging multiple messages together.
* Added example of way to log related events to message store.
* Added examples of id-based operations.
* Added update_id method for updating stored object using ObjectID.
* Added updated time and caller too.
* Added inserter id and time to meta.
* Made wait more obvious
* Working binary with pointclouds.
* Fixed problem with unicode strings in headers.
* updated pkg name ros_mongodb_datacentre to mongodb_store
* Adding delete function to MessageStoreProxy and using it in unittest.
* Adds a service to delete message by ID
* Adding an initial rostest for the message store proxy.
* Returning id correctly from service call.
* Made id query return single element.
* Added ObjectID into meta on query return
* Now returning from query srv
* Added ability to query message store by ObjectId (python only for now).
  Also added some little bits of logging.
* Adding delete function to MessageStoreProxy and using it in unittest.
* Adds a service to delete message by ID
* Adding an initial rostest for the message store proxy.
* Merge pull request `#28 <https://github.com/strands-project/mongodb_store/issues/28>`_ from hawesie/hydro-devel
  Changes for strands_executive
* Returning id correctly from service call.
* Made id query return single element.
* [message_store] fixing query service
* Added ObjectID into meta on query return
* Now returning from query srv
* Added ability to query message store by ObjectId (python only for now).
  Also added some little bits of logging.
* [message-store] Dealing with lists in stored messages. Bug `#25 <https://github.com/strands-project/mongodb_store/issues/25>`_
* fixed update bug where meta info not updated got dropped from the db
* Made sure name is set correctly with using update named.
* All C++ message_store using BSON and meta returns are in json.
  This means that any legal JSON can now be used for a meta description of an object.
* Proof of concept working with C++ BSON library.
* Adding C++ interface for update.
  Fixed compile issues for srv api change.
* Working update method on the python side. Will not work in C++ yet.
* Message store queries now return meta as well as message.
  This is only in the python client for now, but is simple to add to C++. This could be inefficient, so in the future potentially add non-meta options.
* Moved default datacentre path back to /opt/strands
* Switched strands_datacentre to mongodb_store in here.
* Set default database and collection to be message_store.
  We decided to set these in some way I can't quite recall...
* Added message store to launch file.
  Also made HOSTNAME optional.
* C++ queries are working in a basic form.
* C++ query works
* Now using json.dumps and loads to do better queries from python. C++ is still a pain though.
* Query now returns the messages asked for
* Query structure in place
* Meta stuff working on the way in. Starting to think about querying.
* Added meta in agreed format.
* Wrapping python functionality into a class.
* Working across languages with return value now.
* Works in both languages now!
* Working from the C++ end, but this invalidates the Python again.
* Basic insert chain will work in python. Now on to C++.
* Basic idea works python to python
* Service code runs (not working though)
* Adding an insert service and the start of a message store to provide it.
* Changed db path to be more general.
* Updated launch file.
* Moved strands_datacentre to mongodb_store
* Exporting message_store library from package.
* Merge branch 'hydro-devel' of https://github.com/hawesie/mongodb_store into hydro-devel
  Conflicts:
  mongodb_store_cpp_client/CMakeLists.txt
  mongodb_store_cpp_client/include/mongodb_store/message_store.h
* Cleaned up differences between two commits.
* Added updateID to cpp client.
* Added cpp example of logging multiple messages together.
* Changed order of MessageStoreProxy constructor arguments.
  This was done to allow more natural changing of parameters in a sensible order. It's more likely you want to change collection name first, so that is the first parameter, leaving remainder as default.
* Added point cloud test, but not including in compilation.
* Working binary with pointclouds.
* Added updateID to cpp client.
* Added cpp example of logging multiple messages together.
* Changed order of MessageStoreProxy constructor arguments.
  This was done to allow more natural changing of parameters in a sensible order. It's more likely you want to change collection name first, so that is the first parameter, leaving remainder as default.
* Added point cloud test, but not including in compilation.
* Working binary with pointclouds.
* Renamed the library to message_store and moved some files around
* Added a mongodb_store_cpp_client library to avoid multiple definitions of some symbols
* Fixed multiple definition error in C++
* Added rostest launch file.
* Renamed to match convention.
* Added test class for cpp interface.
* Query methods now only return true when something was found.
* Added delete to example script.
* Added constant for empty bson obj.
* Added queryID to C++ side. Insert operations now return IDs. This closes `#29 <https://github.com/strands-project/mongodb_store/issues/29>`_.
  Minor formatting.
* Changed to get the deserialisation length from the vector length.
  This removes the bug where variable length types were incorrectly deserialised. Thanks to @nilsbore for reporting the bug.
* Changed to get the deserialisation length from the vector length.
  This removes the bug where variable length types were incorrectly deserialised. Thanks to @nilsbore for reporting the bug.
* swapping order of target link libraries to fix compiling error
* Made sure name is set correctly with using update named.
* Changed order of parameters for updateNamed to allow people to ignore BSON for as long as possible.
* All C++ message_store using BSON and meta returns are in json.
  This means that any legal JSON can now be used for a meta description of an object.
* Proof of concept working with C++ BSON library.
* Added and tested update interface to C++ side.
* Adding C++ interface for update.
  Fixed compile issues for srv api change.
* Set default database and collection to be message_store.
  We decided to set these in some way I can't quite recall...
* C++ queries are working in a basic form.
* C++ query works
* Query now returns the messages asked for
* Meta stuff working on the way in. Starting to think about querying.
* Renamed file to match python side
* Default values provided
* Moving some functionality to header file for client utils.
* Working from the C++ end, but this invalidates the Python again.
* Contributors: Alex Bencz, Bruno Lacerda, Chris Burbridge, Jaime Pulido Fentanes, Nick Hawes, Thomas Fäulhammer, Rares Ambrus
