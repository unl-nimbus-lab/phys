#ifndef INCLUDED_OKAOSERVER_MESSAGE_H
#define INCLUDED_OKAOSERVER_MESSAGE_H
#include <vector>
#include <string>
#include <iostream>
#include "zmq.hpp"
#include <msgpack.hpp>
namespace OkaoServer
{
  /**
   * @brief サーバへのリクエストメッセージ
   */
  class RequestMessage
  {
  public:
    std::string param;	///< パラメータ（JSON形式）
    std::vector<unsigned char> img;	///< 画像データ
    MSGPACK_DEFINE(param, img);	///< MessagePack用定義
  };
  /**
   * @brief サーバからのリプライメッセージ
   */
  class ReplyMessage
  {
  public:
    std::string okao;	///< OKAO Visionの処理結果（JSON形式）
    MSGPACK_DEFINE(okao);	///< MessagePack用定義
  };
  /**
   * @brief リクエストメッセージの受信処理
   */
  void recvRequestMessage(zmq::socket_t& socket, RequestMessage* reqMsg)
  {
    zmq::message_t request;
    socket.recv(&request);
    msgpack::unpacked msg;
    msgpack::unpack(&msg, reinterpret_cast<const char*>(request.data()), request.size());
    msg.get().convert(reqMsg);
  }
  /**
   * @brief リプライメッセージの送信処理
   */
  void sendReplyMessage(zmq::socket_t& socket, const ReplyMessage& repMsg)
  {
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, repMsg);
    zmq::message_t reply(sbuf.size());
    std::memcpy(reply.data(), sbuf.data(), sbuf.size());
    socket.send(reply);
  }
  /**
   * @brief リクエストメッセージの送信処理（クライアント用）
   */
  void sendRequestMessage(zmq::socket_t& socket, const RequestMessage& reqMsg)
  {
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, reqMsg);
    zmq::message_t request(sbuf.size());
    std::memcpy(request.data(), sbuf.data(), sbuf.size());
    socket.send(request);	
  }
  /**
   * @brief リプライメッセージの受信処理（クライアント用）
   */
  void recvReplyMessage(zmq::socket_t& socket, ReplyMessage* repMsg)
  {
    zmq::message_t reply;
    socket.recv(&reply);
    msgpack::unpacked msg;
    msgpack::unpack(&msg, reinterpret_cast<const char*>(reply.data()), reply.size());
    msg.get().convert(repMsg);
  }
}
#endif 
