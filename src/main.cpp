/* INCLUDES ******************************************************************/

#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Internals/Utils.h"
#include "json.hpp"
#include "ModelPredictiveController.h"
#include "VehicleState.h"

/* USINGS ********************************************************************/

using json = nlohmann::json;

/* DEFINITIONS ***************************************************************/

static const int    POLYNOMIAL_DEGREE                = 3;
static const int    SIMULATOR_PORT                   = 4567;
static const int    WEBSOCKET_MESSAGE_EVENT_MIN_SIZE = 2;
static const int    FUTURE_POINTS_NUMBER             = 25;
static const char   WEBSOCKET_MESSAGE_INDICATOR      = '4';
static const char   WEBSOCKET_EVENT_INDICATOR        = '2';
static const double DT_SECONDS                       = 0.1;
static const double LATENCY_SECONDS                  = 0.1;
static const double LF_VALUE                         = 2.67;
static const double ANGLE_NORMALIZATION_FACTOR       = deg2rad(25) * LF_VALUE;

/* STATIC DECLARATIONS *******************************************************/

static uWS::Hub                  webSocketHub;
static ModelPredictiveController mpc(LF_VALUE, DT_SECONDS);

/* PROTOTYPES ****************************************************************/

/**
 * @brief Verifies if the data is a message event from a websocket.
 *
 * @param data The data from the socket.
 *
 * @return true if the data is a message event from a websocket; otherwise false.
 */
static bool isMessageEvent(const std::string& data);

/**
 * @brief Gets the data from the SocketIO output.
 *
 * @param rawData The data from the socket.
 *
 * @return If there is data, the JSON object in string format will be returned. Otherwise, an empty string
 * will be returned.
 */
static std::string getData(const std::string& rawData);

/**
 * @brief Executes when the websocket is connected.
 *
 * @param webSocket The web socket.
 * @param webSocket The HTTP request.
 */
static void onConnection(uWS::WebSocket<uWS::SERVER> webSocket, uWS::HttpRequest request);

/**
 * @brief Executes when a HTTP request arrives/.
 *
 * @param res         The HTTP response.
 * @param req         The HTTP request.
 * @param data        The data buffer.
 * @param bytesToRead The bytes to read.
 * @param length      The lenght of the response.
 */
static void onHttpRequest(uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t bytesToRead, size_t length);

/**
 * @brief Executes when a message is received by the web socket.
 *
 * @param ws     The web socket.
 * @param data   The message data.
 * @param length The lenght of the data.
 * @param opCode The operation code.
 */
static void onMessage(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode);

/* IMPLEMENTATION ************************************************************/

bool
isMessageEvent(const std::string& data)
{
  const bool hasMinSize = data.size() > WEBSOCKET_MESSAGE_EVENT_MIN_SIZE;
  const bool isMessage  = data[0] == WEBSOCKET_MESSAGE_INDICATOR;
  const bool isEvent    = data[1] == WEBSOCKET_EVENT_INDICATOR;

  return hasMinSize && isMessage && isEvent;
}

std::string
getData(const std::string& rawData)
{
  const bool  foundNull = rawData.find("null") != std::string::npos;
  const auto  b1        = rawData.find_first_of("[");
  const auto  b2        = rawData.rfind("}]");
  std::string result;

  if (foundNull == false && b1 != std::string::npos && b2 != std::string::npos)
    result = rawData.substr(b1, b2 - b1 + 2);

  return result;
}

void
onConnection(uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest)
{
  std::cout << "Connected!!!" << std::endl;
}

void
onDisconnection(uWS::WebSocket<uWS::SERVER> webSocket, int, char*, size_t)
{
  webSocket.close();
  std::cout << "Disconnected" << std::endl;
}

void
onHttpRequest(uWS::HttpResponse* httpResponse, uWS::HttpRequest request, char* data, size_t bytesToRead, size_t length)
{
  const std::string response = "<h1>Hello world!</h1>";

  if (request.getUrl().valueLength == 1)
    httpResponse->end(response.data(), response.length());
  else
    httpResponse->end(nullptr, 0);
}

void
onMessage(uWS::WebSocket<uWS::SERVER> webSocket, char *data, size_t length, uWS::OpCode opCode)
{
  std::string messageData = std::string(data).substr(0, length);

  if (isMessageEvent(messageData)) 
  {
    std::string eventData = getData(messageData);

    if (eventData.empty() == false) 
    {
      auto        data  = json::parse(eventData);
      std::string event = data[0].get<std::string>();

      if (event == "telemetry") 
      {
        std::vector<double> ptsx = data[1]["ptsx"];
        std::vector<double> ptsy = data[1]["ptsy"];

        const double px    = data[1]["x"];
        const double py    = data[1]["y"];
        const double psi   = data[1]["psi"];
        const double v     = data[1]["speed"];
        const double delta = data[1]["steering_angle"];
        const double a     = data[1]["throttle"];

        for (size_t i = 0; i < ptsx.size(); ++i) 
        {
          const double shift_x = ptsx[i] - px;
          const double shift_y = ptsy[i] - py;

          ptsx[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
          ptsy[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
        }

        // Convert to Eigen::VectorXd
        double *ptrx = &ptsx[0];
        double *ptry = &ptsy[0];

        const Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
        const Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

        const auto   coeffs      = polyfit(ptsx_transform, ptsy_transform, POLYNOMIAL_DEGREE);
        const double cte         = polyeval(coeffs, 0);
        const double epsi        = -atan(coeffs[1]);
        const double speedFactor = v * - delta / LF_VALUE * LATENCY_SECONDS;
        const double pred_px     = 0.0  + v * LATENCY_SECONDS;
        const double pred_py     = 0.0;
        const double pred_psi    = 0.0  + speedFactor;
        const double pred_v      = v    + a * LATENCY_SECONDS;
        const double pred_cte    = cte  + v * sin(epsi) * LATENCY_SECONDS;
        const double pred_epsi   = epsi + speedFactor;

        // Feed in the predicted state values
        Eigen::VectorXd state(6);
        state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

        const MpcResult vars = mpc.Solve(state, coeffs);

        // Display the waypoints / reference line
        std::vector<double> next_x_vals;
        std::vector<double> next_y_vals;

        for (int i = 1; i < FUTURE_POINTS_NUMBER; ++i)
        {
            double next_x = i;
            double next_y = polyeval(coeffs, next_x);
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
        }

        // Normalize steering angle range [-deg2rad(25), deg2rad(25] -> [-1, 1].

        const double steer_value    = vars.getSteeringAngle() / ANGLE_NORMALIZATION_FACTOR;
        const double throttle_value = vars.getThrottle();

        //Display the MPC predicted trajectory
        std::vector<double> mpc_x_vals = vars.getPointsX();
        std::vector<double> mpc_y_vals = vars.getPointsY();

        // Compose message for simulator client
        json msgJson;

        msgJson["steering_angle"] = steer_value;
        msgJson["throttle"]       = throttle_value;
        msgJson["mpc_x"]          = mpc_x_vals;
        msgJson["mpc_y"]          = mpc_y_vals;
        msgJson["next_x"]         = next_x_vals;
        msgJson["next_y"]         = next_y_vals;

        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
        //std::cout << msg << std::endl;
        // Latency
        // The purpose is to mimic real driving conditions where
        // the car does actuate the commands instantly.
        //
        // Feel free to play around with this value but should be to drive
        // around the track with 100ms latency.
        //
        // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
        // SUBMITTING.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        webSocket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } 
    else 
    {
      // Manual driving
      std::string msg = "42[\"manual\",{}]";
      webSocket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  }
}

int
main() 
{
  webSocketHub.onHttpRequest(&onHttpRequest);
  webSocketHub.onConnection(&onConnection);
  webSocketHub.onDisconnection(&onDisconnection);
  webSocketHub.onMessage(&onMessage);
  
  if (webSocketHub.listen(SIMULATOR_PORT))
  {
    std::cout << "Listening to port " << SIMULATOR_PORT << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  webSocketHub.run();
}
