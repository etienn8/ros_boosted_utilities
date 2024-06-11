#pragma once

#include "ros/ros.h"

#include <string>

using std::string;

/**
 * @brief Wrapper over a persistent ros::ServiceClient that tries to reconnect to the service if the connection is lost.
 * @tparam TServiceType The service type that the client is going to call.
*/
template <typename TServiceType>
class PersistentServiceClient
{
    public:
    
        /**
         * @brief Defautl constructor that doesn't contain any service client. A PersistentServiceClient should be assigned to it after construction..
        */
        PersistentServiceClient() = default;

        /**
         * @brief Creates a persistent service client that connects to the service with the given name from the given node handle.
         * @param nh The node handle that the service client is going to be created from and that will be used for the reconnexion.
         * @param service_name The name of the service that the client is going to connect to.
        */
        PersistentServiceClient(ros::NodeHandle nh, const std::string& service_name): nh_(nh)
        {
            client_ = nh_.serviceClient<TServiceType>(service_name, true);
        }

        /**
         * @brief Verify that the connection with the service server is still valid and tries to reconnect if it is not. Calls the
         * service it the connection is or becomes valid. If the service is not available, it will wait undefinitely.
         * @param service Reference to the service object that is going to be called. Serves also as the return of the client through its response.
         * @return True if the service call was successful, false otherwise.
        */
        bool call(TServiceType& service)
        {
            if (!client_.isValid())
            {
                const string service_name = client_.getService();

                ROS_WARN_STREAM("Lost connection to service :" << service_name <<". Trying to reconnect and waiting until available.");
                client_ = nh_.serviceClient<TServiceType>(service_name, true);
                client_.waitForExistence();
                ROS_WARN_STREAM("Restored connection to " << service_name);
            }
            return client_.call(service);
        }

        /**
         * @brief Wait of the service to be available.
         * @param timeout The time to wait for the service to be available. If the timeout is negative, it will wait indefinitely.
        */
        bool waitForExistence(ros::Duration timeout = ros::Duration(-1))
        {
            return client_.waitForExistence(timeout);
        }

        /**
         * @brief Verify that the connection with the service server is still valid.
         */
        bool isValid() const
        {
            return client_.isValid();
        }

        /**
         * @brief Shutdown the service client.
        */
        void shutdown()
        {
            client_.shutdown();
        }

        /**
         * @brief Get the name of the service that the client is connected to.
         * @return The name of the service.
        */
        string getService()
        {
            return client_.getService();
        }

    protected:
        /**
         * @brief The underlying ros::ServiceClient that this class is wrapped around.
        */
        ros::ServiceClient client_;

    private:
        /**
         * @brief The node handle that the service client is created from and that will be used for the reconnexion.
        */
        ros::NodeHandle nh_;
};