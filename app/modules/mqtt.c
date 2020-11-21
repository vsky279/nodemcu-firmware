// Module for mqtt
//
#include "module.h"
#include "lauxlib.h"
#include "platform.h"

#include <string.h>
#include <stddef.h>

#include <stdint.h>
#include "mem.h"
#include "lwip/ip_addr.h"
#include "espconn.h"

#include "mqtt/mqtt_msg.h"
#include "mqtt/msg_queue.h"

#include "user_interface.h"

#define MQTT_BUF_SIZE 1460
#define MQTT_DEFAULT_KEEPALIVE 60
#define MQTT_MAX_CLIENT_LEN   64
#define MQTT_MAX_USER_LEN     64
#define MQTT_MAX_PASS_LEN     64
#define MQTT_SEND_TIMEOUT     5

  /*
   * This timeout needs to be long enough for a typical TCP connect()
   * *and* the TLS handshake, if any.  Most network stacks seem to wait
   * tens of seconds for connect(), and TLS can take a good deal of time
   * and several round trips.  Because this matters only rarely, it may
   * as well be set pretty high.
   */
#define MQTT_CONNECT_TIMEOUT  60

typedef enum {
  MQTT_INIT,
  MQTT_CONNECT_SENT,
  MQTT_CONNECT_SENDING,
  MQTT_DATA
} tConnState;

typedef struct mqtt_event_data_t
{
  uint8_t type;
  const char* topic;
  const char* data;
  uint16_t topic_length;
  uint16_t data_length;
  uint16_t data_offset;
} mqtt_event_data_t;

typedef enum {
    MQTT_RECV_NORMAL,
    MQTT_RECV_BUFFERING_SHORT,
    MQTT_RECV_BUFFERING,
    MQTT_RECV_SKIPPING,
} tReceiveState;

typedef struct mqtt_state_t
{
  uint16_t port;
  msg_queue_t* pending_msg_q;
  uint16_t next_message_id;

  uint8_t * recv_buffer; // heap buffer for multi-packet rx
  uint8_t * recv_buffer_wp; // write pointer in multi-packet rx
  union {
      uint16_t recv_buffer_size; // size of recv_buffer
      uint32_t recv_buffer_skip; // number of bytes left to skip, in skipping state
  };
  tReceiveState recv_buffer_state;

} mqtt_state_t;


typedef struct lmqtt_userdata
{
  struct espconn pesp_conn;
  int self_ref;
  int cb_connect_ref;
  int cb_connect_fail_ref;
  int cb_disconnect_ref;
  int cb_message_ref;
  int cb_overflow_ref;
  int cb_suback_ref;
  int cb_unsuback_ref;
  int cb_puback_ref;
  mqtt_state_t  mqtt_state;
  mqtt_connect_info_t connect_info;
  uint16_t keep_alive_tick;
  uint32_t event_timeout;
#ifdef CLIENT_SSL_ENABLE
  uint8_t secure;
#endif
  bool connected;     // indicate socket connected, not mqtt prot connected.
  bool keepalive_sent;
  ETSTimer mqttTimer;
  tConnState connState;
}lmqtt_userdata;

// How large MQTT messages to accept by default
#define DEFAULT_MAX_MESSAGE_LENGTH 1024

static sint8 mqtt_socket_do_connect(struct lmqtt_userdata *);
static void mqtt_socket_reconnected(void *arg, sint8_t err);
static void mqtt_socket_connected(void *arg);
static void mqtt_connack_fail(lmqtt_userdata * mud, int reason_code);

static uint16_t mqtt_next_message_id(lmqtt_userdata * mud)
{
  mud->mqtt_state.next_message_id++;
  if (mud->mqtt_state.next_message_id == 0)
    mud->mqtt_state.next_message_id++;

  return mud->mqtt_state.next_message_id;
}

static void mqtt_socket_disconnected(void *arg)    // tcp only
{
  NODE_DBG("enter mqtt_socket_disconnected.\n");
  bool call_back = false;
  lmqtt_userdata *mud = arg;
  if(mud == NULL)
    return;

  os_timer_disarm(&mud->mqttTimer);

  lua_State *L = lua_getstate();

  if(mud->connected){     // call back only called when socket is from connection to disconnection.
    mud->connected = false;
    if((mud->cb_disconnect_ref != LUA_NOREF) && (mud->self_ref != LUA_NOREF)) {
      lua_rawgeti(L, LUA_REGISTRYINDEX, mud->cb_disconnect_ref);
      lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);  // pass the userdata(client) to callback func in lua
      call_back = true;
    }
  }

  if(mud->mqtt_state.recv_buffer) {
    free(mud->mqtt_state.recv_buffer);
    mud->mqtt_state.recv_buffer = NULL;
  }
  mud->mqtt_state.recv_buffer_size = 0;
  mud->mqtt_state.recv_buffer_state = MQTT_RECV_NORMAL;

  if(mud->pesp_conn.proto.tcp)
    free(mud->pesp_conn.proto.tcp);
  mud->pesp_conn.proto.tcp = NULL;

  mud->connected = false;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->self_ref);
  mud->self_ref = LUA_NOREF; // unref this, and the mqtt.socket userdata will delete it self

  if(call_back){
    lua_call(L, 1, 0);
  }

  NODE_DBG("leave mqtt_socket_disconnected.\n");
}

static void mqtt_socket_do_disconnect(struct lmqtt_userdata *mud)
{
#ifdef CLIENT_SSL_ENABLE
  if (mud->secure) {
    espconn_secure_disconnect(&mud->pesp_conn);
  } else
#endif
  {
    espconn_disconnect(&mud->pesp_conn);
  }
}

static void mqtt_socket_reconnected(void *arg, sint8_t err)
{
  NODE_DBG("enter mqtt_socket_reconnected.\n");
  lmqtt_userdata *mud = arg;
  if(mud == NULL)
    return;

  os_timer_disarm(&mud->mqttTimer);

  mud->event_timeout = 0; // no need to count anymore

  mqtt_socket_do_disconnect(mud);

  mqtt_connack_fail(mud, MQTT_CONN_FAIL_SERVER_NOT_FOUND);

  mqtt_socket_disconnected(arg);
  NODE_DBG("leave mqtt_socket_reconnected.\n");
}

static void deliver_publish(lmqtt_userdata * mud, uint8_t* message, uint16_t length, uint8_t is_overflow)
{
  NODE_DBG("enter deliver_publish (len=%d, overflow=%d).\n", length, is_overflow);
  if(mud == NULL)
    return;
  mqtt_event_data_t event_data;

  event_data.topic_length = length;
  event_data.topic = mqtt_get_publish_topic(message, &event_data.topic_length);

  event_data.data_length = length;
  event_data.data = mqtt_get_publish_data(message, &event_data.data_length);

  int cb_ref = !is_overflow ? mud->cb_message_ref : mud->cb_overflow_ref;

  if(cb_ref == LUA_NOREF)
    return;
  if(mud->self_ref == LUA_NOREF)
    return;
  lua_State *L = lua_getstate();
  if(event_data.topic && (event_data.topic_length > 0)){
    lua_rawgeti(L, LUA_REGISTRYINDEX, cb_ref);
    lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);  // pass the userdata to callback func in lua
    lua_pushlstring(L, event_data.topic, event_data.topic_length);
  } else {
    NODE_DBG("get wrong packet.\n");
    return;
  }
  if(event_data.data && (event_data.data_length > 0)){
    lua_pushlstring(L, event_data.data, event_data.data_length);
    lua_call(L, 3, 0);
  } else {
    lua_call(L, 2, 0);
  }
  NODE_DBG("leave deliver_publish.\n");
}


static void mqtt_connack_fail(lmqtt_userdata * mud, int reason_code)
{
  if(mud->cb_connect_fail_ref == LUA_NOREF || mud->self_ref == LUA_NOREF)
  {
    return;
  }

  lua_State *L = lua_getstate();

  lua_rawgeti(L, LUA_REGISTRYINDEX, mud->cb_connect_fail_ref);
  lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);  // pass the userdata(client) to callback func in lua
  lua_pushinteger(L, reason_code);
  lua_call(L, 2, 0);
}

static sint8 mqtt_send_if_possible(struct lmqtt_userdata *mud)
{
  sint8 espconn_status = ESPCONN_OK;

  // This indicates if we have sent something and are waiting for something to
  // happen
  if (mud->event_timeout == 0) {
    msg_queue_t *pending_msg = msg_peek(&(mud->mqtt_state.pending_msg_q));
    if (pending_msg) {
      mud->event_timeout = MQTT_SEND_TIMEOUT;
      NODE_DBG("Sent: %d\n", pending_msg->msg.length);
#ifdef CLIENT_SSL_ENABLE
      if( mud->secure )
      {
        espconn_status = espconn_secure_send(&mud->pesp_conn, pending_msg->msg.data, pending_msg->msg.length );
      }
      else
#endif
      {
        espconn_status = espconn_send(&mud->pesp_conn, pending_msg->msg.data, pending_msg->msg.length );
      }
      mud->keep_alive_tick = 0;
    }
  }
  NODE_DBG("send_if_poss, queue size: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)));
  return espconn_status;
}

static void mqtt_socket_received(void *arg, char *pdata, unsigned short len)
{
  NODE_DBG("enter mqtt_socket_received (rxlen=%u).\n", len);

  uint8_t msg_type;
  uint8_t msg_qos;
  uint16_t msg_id;
  uint8_t *in_buffer = (uint8_t *)pdata;
  uint16_t in_buffer_length = len;
  uint8_t *continuation_buffer = NULL;
  uint8_t *temp_pdata = NULL;

  lmqtt_userdata *mud = arg;
  if(mud == NULL)
    return;

  switch(mud->mqtt_state.recv_buffer_state) {
    case MQTT_RECV_NORMAL:
      // No previous buffer.
      break;
    case MQTT_RECV_BUFFERING_SHORT:
      // Last buffer had so few byte that we could not determine message length.
      // Store in a local heap buffer and operate on this, as if was the regular pdata buffer.
      // Avoids having to repeat message size/overflow logic.
      temp_pdata = calloc(1,mud->mqtt_state.recv_buffer_size + len);
      if(temp_pdata == NULL) {
        NODE_DBG("MQTT[buffering-short]: Failed to allocate %u bytes, disconnecting...\n", mud->mqtt_state.recv_buffer_size + len);
        mqtt_socket_do_disconnect(mud);
        return;
      }

      NODE_DBG("MQTT[buffering-short]: Continuing with %u + %u bytes\n", mud->mqtt_state.recv_buffer_size, len);
      memcpy(temp_pdata, mud->mqtt_state.recv_buffer, mud->mqtt_state.recv_buffer_size);
      memcpy(temp_pdata + mud->mqtt_state.recv_buffer_size, pdata, len);
      free(mud->mqtt_state.recv_buffer);
      mud->mqtt_state.recv_buffer = NULL;
      mud->mqtt_state.recv_buffer_state = MQTT_RECV_NORMAL;

      in_buffer = temp_pdata;
      in_buffer_length = mud->mqtt_state.recv_buffer_size + len;
      break;

    case MQTT_RECV_BUFFERING: {
      // safe cast: we never allow longer buffer.
      uint16_t current_length = (uint16_t) (mud->mqtt_state.recv_buffer_wp - mud->mqtt_state.recv_buffer);

      NODE_DBG("MQTT[buffering]: appending %u bytes to previous recv buffer (%u out of wanted %u)\n",
               in_buffer_length,
               current_length,
               mud->mqtt_state.recv_buffer_size);

      // Copy from rx buffer to heap buffer. Smallest of [remainder of pending message] and [all of buffer]
      uint16_t copy_length = LWIP_MIN(mud->mqtt_state.recv_buffer_size - current_length, in_buffer_length);
      memcpy(mud->mqtt_state.recv_buffer_wp, pdata, copy_length);
      mud->mqtt_state.recv_buffer_wp += copy_length;

      in_buffer_length = (uint16_t) (mud->mqtt_state.recv_buffer_wp - mud->mqtt_state.recv_buffer);
      if (in_buffer_length < mud->mqtt_state.recv_buffer_size) {
        NODE_DBG("MQTT[buffering]: need %u more bytes, waiting for next rx.\n",
                 mud->mqtt_state.recv_buffer_size - in_buffer_length
                 );
        goto RX_PACKET_FINISHED;
      }

      NODE_DBG("MQTT[buffering]: Full message received (%u). remainding bytes=%u\n",
               mud->mqtt_state.recv_buffer_size,
               len - copy_length);

      // Point continuation_buffer to any additional data in pdata.
      // May become 0 bytes, but used to trigger free!
      continuation_buffer = pdata + copy_length;
      len -= copy_length; // borrow len instead of having another variable..

      in_buffer = mud->mqtt_state.recv_buffer;
      // in_buffer_length was set above
      mud->mqtt_state.recv_buffer_state = MQTT_RECV_NORMAL;

      break;
    }
    case MQTT_RECV_SKIPPING:
      // Last rx had a message which was too large to process, skip it.
      if(mud->mqtt_state.recv_buffer_skip > in_buffer_length) {
        NODE_DBG("MQTT[skipping]: skip=%u. Skipping full RX buffer (%u).\n",
                 mud->mqtt_state.recv_buffer_skip,
                 in_buffer_length
            );
        mud->mqtt_state.recv_buffer_skip -= in_buffer_length;
        goto RX_PACKET_FINISHED;
      }

      NODE_DBG("MQTT[skipping]: skip=%u. Skipping partial RX buffer, continuing at %u\n",
               mud->mqtt_state.recv_buffer_skip,
               in_buffer_length
      );

      in_buffer += mud->mqtt_state.recv_buffer_skip;
      in_buffer_length -= mud->mqtt_state.recv_buffer_skip;

      mud->mqtt_state.recv_buffer_skip = 0;
      mud->mqtt_state.recv_buffer_state = MQTT_RECV_NORMAL;
      break;
  }

READPACKET:
  if(in_buffer_length <= 0)
    goto RX_PACKET_FINISHED;

  // MQTT publish message can in theory be 256Mb, while we do not support it we need to be
  // able to do math on it.
  int32_t message_length;

  // temp buffer for control messages
  uint8_t temp_buffer[MQTT_BUF_SIZE];
  mqtt_message_buffer_t msgb;
  mqtt_msg_init(&msgb, temp_buffer, MQTT_BUF_SIZE);
  mqtt_message_t *temp_msg = NULL;

  lua_State *L = lua_getstate();
  switch(mud->connState){
    case MQTT_CONNECT_SENDING:
    case MQTT_CONNECT_SENT:
      mud->event_timeout = 0;

      if(mqtt_get_type(in_buffer) != MQTT_MSG_TYPE_CONNACK){
        NODE_DBG("MQTT: Invalid packet\r\n");
        mud->connState = MQTT_INIT;
        mqtt_socket_do_disconnect(mud);
        mqtt_connack_fail(mud, MQTT_CONN_FAIL_NOT_A_CONNACK_MSG);
        break;

      } else if (mqtt_get_connect_ret_code(in_buffer) != MQTT_CONNACK_ACCEPTED) {
        NODE_DBG("MQTT: CONNACK REFUSED (CODE: %d)\n", mqtt_get_connect_ret_code(in_buffer));

        mud->connState = MQTT_INIT;

        mqtt_socket_do_disconnect(mud);
        mqtt_connack_fail(mud, mqtt_get_connect_ret_code(in_buffer));
        break;

      } else {
        mud->connState = MQTT_DATA;
        NODE_DBG("MQTT: Connected\r\n");
        mud->keepalive_sent = 0;
        if(mud->cb_connect_ref == LUA_NOREF)
          break;
        if(mud->self_ref == LUA_NOREF)
          break;
        lua_rawgeti(L, LUA_REGISTRYINDEX, mud->cb_connect_ref);
        lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);  // pass the userdata(client) to callback func in lua
        lua_call(L, 1, 0);
        break;
      }
      break;

    case MQTT_DATA:
      message_length = mqtt_get_total_length(in_buffer, in_buffer_length);
      msg_type = mqtt_get_type(in_buffer);
      msg_qos = mqtt_get_qos(in_buffer);
      msg_id = mqtt_get_id(in_buffer, in_buffer_length);

      NODE_DBG("MQTT_DATA: msg length: %u, buffer length: %u\r\n",
           message_length,
           in_buffer_length);

      if (message_length > mud->connect_info.max_message_length) {
        // The pending message length is larger than we was configured to allow
        if(msg_qos > 0 && msg_id == 0) {
          NODE_DBG("MQTT: msg too long, but not enough data to get msg_id: total=%u, deliver=%u\r\n", message_length, in_buffer_length);
          // qos requested, but too short buffer to get a packet ID.
          // Trigger the "short buffer" mode
          message_length = -1;
          // Drop through to partial message handling below.
        } else {
          NODE_DBG("MQTT: msg too long: total=%u, deliver=%u\r\n", message_length, in_buffer_length);
          if (msg_type == MQTT_MSG_TYPE_PUBLISH) {
            // In practice we should never get any other types..
            deliver_publish(mud, in_buffer, in_buffer_length, 1);

            // If qos specified, we should ACK it.
            // In theory it might be wrong to ack it before we received all TCP packets, but this avoids
            // buffering and special code to handle this corner-case. Server will most likely have
            // written all to OS socket anyway, and not be aware that we "should" not have received it all yet.
            if(msg_qos == 1){
              temp_msg = mqtt_msg_puback(&msgb, msg_id);
              msg_enqueue(&(mud->mqtt_state.pending_msg_q), temp_msg,
                          msg_id, MQTT_MSG_TYPE_PUBACK, (int)mqtt_get_qos(temp_msg->data) );
            }
            else if(msg_qos == 2){
              temp_msg = mqtt_msg_pubrec(&msgb, msg_id);
              msg_enqueue(&(mud->mqtt_state.pending_msg_q), temp_msg,
                          msg_id, MQTT_MSG_TYPE_PUBREC, (int)mqtt_get_qos(temp_msg->data) );
            }
            if(msg_qos == 1 || msg_qos == 2){
              NODE_DBG("MQTT: Queue response QoS: %d\r\n", msg_qos);
            }
          }

          if (message_length > in_buffer_length) {
            // Ignore bytes in subsequent packet(s) too.
            NODE_DBG("MQTT: skipping into next rx\n");
            mud->mqtt_state.recv_buffer_state = MQTT_RECV_SKIPPING;
            mud->mqtt_state.recv_buffer_skip = (uint32_t) message_length - in_buffer_length;
            break;
          } else {
            NODE_DBG("MQTT: Skipping message\n");
            mud->mqtt_state.recv_buffer_state = MQTT_RECV_NORMAL;
            goto RX_MESSAGE_PROCESSED;
          }
        }
      }

      if (message_length == -1 || message_length > in_buffer_length) {
        // Partial message in buffer, need to store on heap until next RX. Allocate size for full message directly,
        // instead of potential reallocs, to avoid fragmentation.
        // If message_length is indicated as -1, we do not have enough data to determine the length properly.
        // Just put what we have on heap, and place in state BUFFERING_SHORT.
        NODE_DBG("MQTT: Partial message received (%u of %d). Buffering\r\n",
            in_buffer_length,
            message_length);

        // although message_length is 32bit, it should never go above 16bit since
        // max_message_length is 16bit.
        uint16_t alloc_size = message_length > 0 ? (uint16_t)message_length : in_buffer_length;

        mud->mqtt_state.recv_buffer = calloc(1,alloc_size);
        if (mud->mqtt_state.recv_buffer == NULL) {
          NODE_DBG("MQTT: Failed to allocate %u bytes, disconnecting...\n", alloc_size);
          mqtt_socket_do_disconnect(mud);
          return;
        }

        memcpy(mud->mqtt_state.recv_buffer, in_buffer, in_buffer_length);
        mud->mqtt_state.recv_buffer_wp = mud->mqtt_state.recv_buffer + in_buffer_length;
        mud->mqtt_state.recv_buffer_state = message_length > 0 ? MQTT_RECV_BUFFERING : MQTT_RECV_BUFFERING_SHORT;
        mud->mqtt_state.recv_buffer_size = alloc_size;

        NODE_DBG("MQTT: Wait for next recv\n");
        break;
      }

      msg_queue_t *pending_msg = msg_peek(&(mud->mqtt_state.pending_msg_q));
      NODE_DBG("MQTT_DATA: type: %d, qos: %d, msg_id: %d, pending_id: %d, msg length: %u, buffer length: %u\r\n",
               msg_type,
               msg_qos,
               msg_id,
               (pending_msg)?pending_msg->msg_id:0,
               message_length,
               in_buffer_length);

      switch(msg_type)
      {
        case MQTT_MSG_TYPE_SUBACK:
          if(pending_msg && pending_msg->msg_type == MQTT_MSG_TYPE_SUBSCRIBE && pending_msg->msg_id == msg_id){
            NODE_DBG("MQTT: Subscribe successful\r\n");
            msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
            if (mud->cb_suback_ref == LUA_NOREF)
              break;
            if (mud->self_ref == LUA_NOREF)
              break;
            lua_rawgeti(L, LUA_REGISTRYINDEX, mud->cb_suback_ref);
            lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);
            lua_call(L, 1, 0);
          }
          break;
        case MQTT_MSG_TYPE_UNSUBACK:
          if(pending_msg && pending_msg->msg_type == MQTT_MSG_TYPE_UNSUBSCRIBE && pending_msg->msg_id == msg_id){
            NODE_DBG("MQTT: UnSubscribe successful\r\n");
            msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));

            if (mud->cb_unsuback_ref == LUA_NOREF)
              break;
            if (mud->self_ref == LUA_NOREF)
              break;
            lua_rawgeti(L, LUA_REGISTRYINDEX, mud->cb_unsuback_ref);
            lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);
            lua_call(L, 1, 0);
          }
          break;
        case MQTT_MSG_TYPE_PUBLISH:
          if(msg_qos == 1){
            temp_msg = mqtt_msg_puback(&msgb, msg_id);
            msg_enqueue(&(mud->mqtt_state.pending_msg_q), temp_msg,
                      msg_id, MQTT_MSG_TYPE_PUBACK, (int)mqtt_get_qos(temp_msg->data) );
          }
          else if(msg_qos == 2){
            temp_msg = mqtt_msg_pubrec(&msgb, msg_id);
            msg_enqueue(&(mud->mqtt_state.pending_msg_q), temp_msg,
                      msg_id, MQTT_MSG_TYPE_PUBREC, (int)mqtt_get_qos(temp_msg->data) );
          }
          if(msg_qos == 1 || msg_qos == 2){
            NODE_DBG("MQTT: Queue response QoS: %d\r\n", msg_qos);
          }
          deliver_publish(mud, in_buffer, (uint16_t)message_length, 0);
          break;
        case MQTT_MSG_TYPE_PUBACK:
          if(pending_msg && pending_msg->msg_type == MQTT_MSG_TYPE_PUBLISH && pending_msg->msg_id == msg_id){
            NODE_DBG("MQTT: Publish with QoS = 1 successful\r\n");
            msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
            if(mud->cb_puback_ref == LUA_NOREF)
              break;
            if(mud->self_ref == LUA_NOREF)
              break;
            lua_rawgeti(L, LUA_REGISTRYINDEX, mud->cb_puback_ref);
            lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);  // pass the userdata to callback func in lua
            lua_call(L, 1, 0);
          }

          break;
        case MQTT_MSG_TYPE_PUBREC:
          if(pending_msg && pending_msg->msg_type == MQTT_MSG_TYPE_PUBLISH && pending_msg->msg_id == msg_id){
            NODE_DBG("MQTT: Publish  with QoS = 2 Received PUBREC\r\n");
            // Note: actually, should not destroy the msg until PUBCOMP is received.
            msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
            temp_msg = mqtt_msg_pubrel(&msgb, msg_id);
            msg_enqueue(&(mud->mqtt_state.pending_msg_q), temp_msg,
                      msg_id, MQTT_MSG_TYPE_PUBREL, (int)mqtt_get_qos(temp_msg->data) );
            NODE_DBG("MQTT: Response PUBREL\r\n");
          }
          break;
        case MQTT_MSG_TYPE_PUBREL:
          if(pending_msg && pending_msg->msg_type == MQTT_MSG_TYPE_PUBREC && pending_msg->msg_id == msg_id){
            msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
            temp_msg = mqtt_msg_pubcomp(&msgb, msg_id);
            msg_enqueue(&(mud->mqtt_state.pending_msg_q), temp_msg,
                      msg_id, MQTT_MSG_TYPE_PUBCOMP, (int)mqtt_get_qos(temp_msg->data) );
            NODE_DBG("MQTT: Response PUBCOMP\r\n");
          }
          break;
        case MQTT_MSG_TYPE_PUBCOMP:
          if(pending_msg && pending_msg->msg_type == MQTT_MSG_TYPE_PUBREL && pending_msg->msg_id == msg_id){
            NODE_DBG("MQTT: Publish  with QoS = 2 successful\r\n");
            msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
            if(mud->cb_puback_ref == LUA_NOREF)
              break;
            if(mud->self_ref == LUA_NOREF)
              break;
            lua_rawgeti(L, LUA_REGISTRYINDEX, mud->cb_puback_ref);
            lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);  // pass the userdata to callback func in lua
            lua_call(L, 1, 0);
          }
          break;
        case MQTT_MSG_TYPE_PINGREQ:
            temp_msg = mqtt_msg_pingresp(&msgb);
            msg_enqueue(&(mud->mqtt_state.pending_msg_q), temp_msg,
                      msg_id, MQTT_MSG_TYPE_PINGRESP, (int)mqtt_get_qos(temp_msg->data) );
            NODE_DBG("MQTT: Response PINGRESP\r\n");
          break;
        case MQTT_MSG_TYPE_PINGRESP:
          // Ignore
          mud->keepalive_sent = 0;
          NODE_DBG("MQTT: PINGRESP received\r\n");
          break;
      }

RX_MESSAGE_PROCESSED:
      if(continuation_buffer != NULL) {
        NODE_DBG("MQTT[buffering]: buffered message finished. Continuing with rest of rx buffer (%u)\n",
                 len);
        free(mud->mqtt_state.recv_buffer);
        mud->mqtt_state.recv_buffer = NULL;

        in_buffer = continuation_buffer;
        in_buffer_length = len;
        continuation_buffer = NULL;
      }else{
        // Message have been fully processed (or ignored). Move pointer ahead
        // and continue with next message, if any.
        in_buffer_length -= message_length;
        in_buffer += message_length;
      }

      if(in_buffer_length > 0)
      {
        NODE_DBG("Get another published message\r\n");
        goto READPACKET;
      }

      break;
  }

RX_PACKET_FINISHED:
  if(temp_pdata != NULL) {
    free(temp_pdata);
  }

  mqtt_send_if_possible(mud);
  NODE_DBG("leave mqtt_socket_received\n");
  return;
}

static void mqtt_socket_sent(void *arg)
{
  NODE_DBG("enter mqtt_socket_sent.\n");
  lmqtt_userdata *mud = arg;
  if(mud == NULL)
    return;
  if(!mud->connected)
    return;
  // call mqtt_sent()
  mud->event_timeout = 0;
  mud->keep_alive_tick = 0;

  if(mud->connState == MQTT_CONNECT_SENDING){
    mud->connState = MQTT_CONNECT_SENT;
    mud->event_timeout = MQTT_SEND_TIMEOUT;
    // MQTT_CONNECT not queued.
    return;
  }
  NODE_DBG("sent1, queue size: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)));
  uint8_t try_send = 1;
  // qos = 0, publish and forgot.
  msg_queue_t *node = msg_peek(&(mud->mqtt_state.pending_msg_q));
  if(node && node->msg_type == MQTT_MSG_TYPE_PUBLISH && node->publish_qos == 0) {
    msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
    if(mud->cb_puback_ref != LUA_NOREF && mud->self_ref != LUA_NOREF) {
      lua_State *L = lua_getstate();
      lua_rawgeti(L, LUA_REGISTRYINDEX, mud->cb_puback_ref);
      lua_rawgeti(L, LUA_REGISTRYINDEX, mud->self_ref);  // pass the userdata to callback func in lua
      lua_call(L, 1, 0);
    }
  } else if(node && node->msg_type == MQTT_MSG_TYPE_PUBACK) {
    msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
  } else if(node && node->msg_type == MQTT_MSG_TYPE_PUBCOMP) {
    msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
  } else if(node && node->msg_type == MQTT_MSG_TYPE_PINGREQ) {
    msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
  } else {
    try_send = 0;
  }
  if (try_send) {
    mqtt_send_if_possible(mud);
  }
  NODE_DBG("sent2, queue size: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)));
  NODE_DBG("leave mqtt_socket_sent.\n");
}

static void mqtt_socket_connected(void *arg)
{
  NODE_DBG("enter mqtt_socket_connected.\n");
  lmqtt_userdata *mud = arg;
  if(mud == NULL)
    return;
  struct espconn *pesp_conn = &mud->pesp_conn;
  mud->connected = true;
  espconn_regist_recvcb(pesp_conn, mqtt_socket_received);
  espconn_regist_sentcb(pesp_conn, mqtt_socket_sent);
  espconn_regist_disconcb(pesp_conn, mqtt_socket_disconnected);

  uint8_t temp_buffer[MQTT_BUF_SIZE];
  mqtt_message_buffer_t msgb;
  mqtt_msg_init(&msgb, temp_buffer, MQTT_BUF_SIZE);

  mqtt_message_t* temp_msg = mqtt_msg_connect(&msgb, &mud->connect_info);
  NODE_DBG("Send MQTT connection infomation, data len: %d, d[0]=%d \r\n", temp_msg->length,  temp_msg->data[0]);

  mud->event_timeout = MQTT_SEND_TIMEOUT;
  // not queue this message. should send right now. or should enqueue this before head.
#ifdef CLIENT_SSL_ENABLE
  if(mud->secure)
  {
    espconn_secure_send(pesp_conn, temp_msg->data, temp_msg->length);
  }
  else
#endif
  {
    espconn_send(pesp_conn, temp_msg->data, temp_msg->length);
  }
  mud->keep_alive_tick = 0;

  mud->connState = MQTT_CONNECT_SENDING;
  NODE_DBG("leave mqtt_socket_connectet, heap = %u.\n", system_get_free_heap_size());
  return;
}

void mqtt_socket_timer(void *arg)
{
  NODE_DBG("enter mqtt_socket_timer.\n");
  lmqtt_userdata *mud = (lmqtt_userdata*) arg;

  if(mud == NULL)
    return;

  if(mud->pesp_conn.proto.tcp == NULL){
    NODE_DBG("MQTT not connected\n");
    os_timer_disarm(&mud->mqttTimer);
    return;
  }

  NODE_DBG("timer, queue size: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)));
  if(mud->event_timeout > 0){
    NODE_DBG("event_timeout: %d.\n", mud->event_timeout);
        mud->event_timeout --;
    if(mud->event_timeout > 0){
      return;
    } else {
      NODE_DBG("event timeout. \n");
      if(mud->connState == MQTT_DATA)
        msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
      // should remove the head of the queue and re-send with DUP = 1
      // Not implemented yet.
    }
  }

  if(mud->connState == MQTT_INIT){ // socket connect time out.
    NODE_DBG("Can not connect to broker.\n");
    os_timer_disarm(&mud->mqttTimer);
    mqtt_socket_do_disconnect(mud);
    mqtt_connack_fail(mud, MQTT_CONN_FAIL_SERVER_NOT_FOUND);
  } else if(mud->connState == MQTT_CONNECT_SENDING){ // MQTT_CONNECT send time out.
    NODE_DBG("sSend MQTT_CONNECT failed.\n");
    mud->connState = MQTT_INIT;
    mqtt_socket_do_disconnect(mud);
    mqtt_connack_fail(mud, MQTT_CONN_FAIL_TIMEOUT_SENDING);

    mud->keep_alive_tick = 0; // not need count anymore
  } else if(mud->connState == MQTT_CONNECT_SENT) { // wait for CONACK time out.
    NODE_DBG("MQTT_CONNECT timeout.\n");
    mud->connState = MQTT_INIT;
    mqtt_socket_do_disconnect(mud);
    mqtt_connack_fail(mud, MQTT_CONN_FAIL_TIMEOUT_RECEIVING);
  } else if(mud->connState == MQTT_DATA){
    msg_queue_t *pending_msg = msg_peek(&(mud->mqtt_state.pending_msg_q));
    if(pending_msg){
      mqtt_send_if_possible(mud);
    } else {
      // no queued event.
      mud->keep_alive_tick ++;
      if(mud->keep_alive_tick > mud->connect_info.keepalive){
        if (mud->keepalive_sent) {
          // Oh dear -- keepalive timer expired and still no ack of previous message
          mqtt_socket_reconnected(&mud->pesp_conn, 0);
        } else {
          uint8_t temp_buffer[MQTT_BUF_SIZE];
          mqtt_message_buffer_t msgb;
          mqtt_msg_init(&msgb, temp_buffer, MQTT_BUF_SIZE);

          NODE_DBG("\r\nMQTT: Send keepalive packet\r\n");
          mqtt_message_t* temp_msg = mqtt_msg_pingreq(&msgb);
          msg_queue_t *node = msg_enqueue( &(mud->mqtt_state.pending_msg_q), temp_msg,
                              0, MQTT_MSG_TYPE_PINGREQ, (int)mqtt_get_qos(temp_msg->data) );
          mud->keepalive_sent = 1;
          mud->keep_alive_tick = 0;     // Need to reset to zero in case flow control stopped.
          mqtt_send_if_possible(mud);
        }
      }
    }
  }
  NODE_DBG("keep_alive_tick: %d\n", mud->keep_alive_tick);
  NODE_DBG("leave mqtt_socket_timer.\n");
}

// Lua: mqtt.Client(clientid, keepalive, user, pass, clean_session, max_message_length)
static int mqtt_socket_client( lua_State* L )
{
  NODE_DBG("enter mqtt_socket_client.\n");

  lmqtt_userdata *mud;
  char tempid[20] = {0};
  sprintf(tempid, "%s%x", "NodeMCU_", system_get_chip_id() );
  NODE_DBG(tempid);
  NODE_DBG("\n");

  const char *clientId = tempid, *username = NULL, *password = NULL;
  size_t idl = strlen(tempid);
  size_t unl = 0, pwl = 0;
  int keepalive = 0;
  int stack = 1;
  int clean_session = 1;
  int max_message_length = 0;
  int top = lua_gettop(L);

  // create a object
  mud = (lmqtt_userdata *)lua_newuserdata(L, sizeof(lmqtt_userdata));
  memset(mud, 0, sizeof(*mud));
  // pre-initialize it, in case of errors
  mud->self_ref = LUA_NOREF;
  mud->cb_connect_ref = LUA_NOREF;
  mud->cb_connect_fail_ref = LUA_NOREF;
  mud->cb_disconnect_ref = LUA_NOREF;

  mud->cb_message_ref = LUA_NOREF;
  mud->cb_overflow_ref = LUA_NOREF;
  mud->cb_suback_ref = LUA_NOREF;
  mud->cb_unsuback_ref = LUA_NOREF;
  mud->cb_puback_ref = LUA_NOREF;

  mud->connState = MQTT_INIT;

  // set its metatable
  luaL_getmetatable(L, "mqtt.socket");
  lua_setmetatable(L, -2);

  if( lua_isstring(L,stack) )   // deal with the clientid string
  {
    clientId = luaL_checklstring( L, stack, &idl );
    stack++;
  }

  if(lua_isnumber( L, stack ))
  {
    keepalive = luaL_checkinteger( L, stack);
    stack++;
  }

  if(keepalive == 0){
    keepalive = MQTT_DEFAULT_KEEPALIVE;
  }

  if(lua_isstring( L, stack )){
    username = luaL_checklstring( L, stack, &unl );
    stack++;
  }
  if(username == NULL)
    unl = 0;
  NODE_DBG("length username: %d\r\n", unl);

  if(lua_isstring( L, stack )){
    password = luaL_checklstring( L, stack, &pwl );
    stack++;
  }
  if(password == NULL)
    pwl = 0;
  NODE_DBG("length password: %d\r\n", pwl);

  if(lua_isnumber( L, stack ))
  {
    clean_session = luaL_checkinteger( L, stack);
    stack++;
  }

  if(clean_session > 1){
    clean_session = 1;
  }

  if(lua_isnumber( L, stack ))
  {
      max_message_length = luaL_checkinteger( L, stack);
      stack++;
  }

  if(max_message_length == 0) {
    max_message_length = DEFAULT_MAX_MESSAGE_LENGTH;
  }

  // TODO: check the zalloc result.
  mud->connect_info.client_id = (uint8_t *)calloc(1,idl+1);
  mud->connect_info.username = (uint8_t *)calloc(1,unl + 1);
  mud->connect_info.password = (uint8_t *)calloc(1,pwl + 1);
  if(!mud->connect_info.client_id || !mud->connect_info.username || !mud->connect_info.password){
    if(mud->connect_info.client_id) {
      free(mud->connect_info.client_id);
      mud->connect_info.client_id = NULL;
    }
    if(mud->connect_info.username) {
      free(mud->connect_info.username);
      mud->connect_info.username = NULL;
    }
    if(mud->connect_info.password) {
      free(mud->connect_info.password);
      mud->connect_info.password = NULL;
    }
    return luaL_error(L, "not enough memory");
  }

  memcpy(mud->connect_info.client_id, clientId, idl);
  mud->connect_info.client_id[idl] = 0;
  memcpy(mud->connect_info.username, username, unl);
  mud->connect_info.username[unl] = 0;
  memcpy(mud->connect_info.password, password, pwl);
  mud->connect_info.password[pwl] = 0;

  NODE_DBG("MQTT: Init info: %s, %s, %s\r\n", mud->connect_info.client_id, mud->connect_info.username, mud->connect_info.password);

  mud->connect_info.clean_session = clean_session;
  mud->connect_info.will_qos = 0;
  mud->connect_info.will_retain = 0;
  mud->connect_info.keepalive = keepalive;
  mud->connect_info.max_message_length = max_message_length;

  mud->mqtt_state.pending_msg_q = NULL;
  mud->mqtt_state.port = 1883;
  mud->mqtt_state.recv_buffer = NULL;
  mud->mqtt_state.recv_buffer_size = 0;
  mud->mqtt_state.recv_buffer_state = MQTT_RECV_NORMAL;

  NODE_DBG("leave mqtt_socket_client.\n");
  return 1;
}

// Lua: mqtt.delete( socket )
// call close() first
// socket: unref everything
static int mqtt_delete( lua_State* L )
{
  NODE_DBG("enter mqtt_delete.\n");

  lmqtt_userdata *mud = (lmqtt_userdata *)luaL_checkudata(L, 1, "mqtt.socket");
  luaL_argcheck(L, mud, 1, "mqtt.socket expected");
  if(mud==NULL){
    NODE_DBG("userdata is nil.\n");
    return 0;
  }

  os_timer_disarm(&mud->mqttTimer);
  mud->connected = false;

  // ---- alloc-ed in mqtt_socket_connect()
  if(mud->pesp_conn.proto.tcp)
    free(mud->pesp_conn.proto.tcp);
  mud->pesp_conn.proto.tcp = NULL;

  while(mud->mqtt_state.pending_msg_q) {
    msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
  }

  // ---- alloc-ed in mqtt_socket_lwt()
  if(mud->connect_info.will_topic){
        free(mud->connect_info.will_topic);
        mud->connect_info.will_topic = NULL;
  }

  if(mud->connect_info.will_message){
    free(mud->connect_info.will_message);
    mud->connect_info.will_message = NULL;
  }
  // ----

  //--------- alloc-ed in mqtt_socket_received()
  if(mud->mqtt_state.recv_buffer) {
    free(mud->mqtt_state.recv_buffer);
    mud->mqtt_state.recv_buffer = NULL;
  }
  // ----

  //--------- alloc-ed in mqtt_socket_client()
  if(mud->connect_info.client_id){
    free(mud->connect_info.client_id);
    mud->connect_info.client_id = NULL;
  }
  if(mud->connect_info.username){
    free(mud->connect_info.username);
    mud->connect_info.username = NULL;
  }
  if(mud->connect_info.password){
    free(mud->connect_info.password);
    mud->connect_info.password = NULL;
  }
  // -------

  // free (unref) callback ref
  luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_connect_ref);
  mud->cb_connect_ref = LUA_NOREF;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_connect_fail_ref);
  mud->cb_connect_fail_ref = LUA_NOREF;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_disconnect_ref);
  mud->cb_disconnect_ref = LUA_NOREF;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_message_ref);
  mud->cb_message_ref = LUA_NOREF;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_overflow_ref);
  mud->cb_overflow_ref = LUA_NOREF;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_suback_ref);
  mud->cb_suback_ref = LUA_NOREF;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_unsuback_ref);
  mud->cb_unsuback_ref = LUA_NOREF;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_puback_ref);
  mud->cb_puback_ref = LUA_NOREF;

  int selfref = mud->self_ref;
  mud->self_ref = LUA_NOREF;
  luaL_unref(L, LUA_REGISTRYINDEX, mud->self_ref);

  NODE_DBG("leave mqtt_delete.\n");
  return 0;
}

static sint8 mqtt_socket_do_connect(struct lmqtt_userdata *mud)
{

  NODE_DBG("enter socket_connect.\n");
  sint8 espconn_status;

  mud->event_timeout = MQTT_CONNECT_TIMEOUT;
  mud->connState = MQTT_INIT;
#ifdef CLIENT_SSL_ENABLE
  if(mud->secure)
  {
    espconn_status = espconn_secure_connect(&mud->pesp_conn);
  }
  else
#endif
  {
    espconn_status = espconn_connect(&mud->pesp_conn);
  }

  os_timer_arm(&mud->mqttTimer, 1000, 1);

  NODE_DBG("leave socket_connect\n");

  return espconn_status;
}

static sint8 socket_dns_found(const char *name, ip_addr_t *ipaddr, void *arg)
{
  lmqtt_userdata *mud = arg;

  NODE_DBG("enter socket_dns_found.\n");
  sint8 espconn_status = ESPCONN_OK;

  if(ipaddr == NULL)
  {
    mqtt_connack_fail(mud, MQTT_CONN_FAIL_DNS);

    // although not connected, but fire disconnect callback to release every thing.
    mqtt_socket_disconnected(arg);
    return -1;
  }

  // ipaddr->addr is a uint32_t ip
  if(ipaddr->addr != 0)
  {
    memcpy(&mud->pesp_conn.proto.tcp->remote_ip, &(ipaddr->addr), 4);
    NODE_DBG("TCP ip is set: ");
    NODE_DBG(IPSTR, IP2STR(&(ipaddr->addr)));
    NODE_DBG("\n");
    espconn_status = mqtt_socket_do_connect(mud);
  }

  NODE_DBG("leave socket_dns_found.\n");

  return espconn_status;
}

/* wrapper for using socket_dns_found() as callback function */
static void socket_dns_foundcb(const char *name, ip_addr_t *ipaddr, void *arg)
{
  socket_dns_found(name, ipaddr, arg);
}

#include "pm/swtimer.h"
// Lua: mqtt:connect( host, port, secure, function(client), function(client, connect_return_code) )
static int mqtt_socket_connect( lua_State* L )
{
  NODE_DBG("enter mqtt_socket_connect.\n");
  lmqtt_userdata *mud = NULL;
  unsigned port = 1883;
  size_t il;
  ip_addr_t ipaddr;
  const char *domain;
  int stack = 1;
  unsigned secure = 0;
  int top = lua_gettop(L);

  mud = (lmqtt_userdata *)luaL_checkudata(L, stack, "mqtt.socket");
  luaL_argcheck(L, mud, stack, "mqtt.socket expected");
  stack++;
  if(mud == NULL)
    return 0;

  if(mud->connected){
    return luaL_error(L, "already connected");
  }

  struct espconn *pesp_conn = &mud->pesp_conn;

  if (!pesp_conn->proto.tcp)
    pesp_conn->proto.tcp = (esp_tcp *)calloc(1,sizeof(esp_tcp));

  if(!pesp_conn->proto.tcp) {
    return luaL_error(L, "not enough memory");
  }

  // reverse is for the callback function
  pesp_conn->type = ESPCONN_TCP;
  pesp_conn->state = ESPCONN_NONE;
  mud->connected = false;

  if( (stack<=top) && lua_isstring(L,stack) )   // deal with the domain string
  {
    domain = luaL_checklstring( L, stack, &il );

    stack++;
    if (domain == NULL)
    {
      domain = "127.0.0.1";
    }
    ipaddr.addr = ipaddr_addr(domain);
    memcpy(pesp_conn->proto.tcp->remote_ip, &ipaddr.addr, 4);
    NODE_DBG("TCP ip is set: ");
    NODE_DBG(IPSTR, IP2STR(&ipaddr.addr));
    NODE_DBG("\n");
  }

  if ( (stack<=top) && lua_isnumber(L, stack) )
  {
    port = lua_tointeger(L, stack);
    stack++;
    NODE_DBG("TCP port is set: %d.\n", port);
  }
  pesp_conn->proto.tcp->remote_port = port;
  if (pesp_conn->proto.tcp->local_port == 0)
    pesp_conn->proto.tcp->local_port = espconn_port();
  mud->mqtt_state.port = port;

  if ( (stack<=top) && (lua_isnumber(L, stack) || lua_isboolean(L, stack)) )
  {
    if (lua_isnumber(L, stack)) {
      platform_print_deprecation_note("mqtt.connect secure parameter as integer","in the future");
      secure = !!lua_tointeger(L, stack);
    } else {
      secure = lua_toboolean(L, stack);
    }
    stack++;
  } else {
    secure = 0; // default to 0
  }
#ifdef CLIENT_SSL_ENABLE
  mud->secure = secure; // save
#else
  if ( secure )
  {
    return luaL_error(L, "ssl not available");
  }
#endif

  // call back function when a connection is obtained, tcp only
  if ((stack<=top) && (lua_isfunction(L, stack))){
    lua_pushvalue(L, stack);  // copy argument (func) to the top of stack
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_connect_ref);
    mud->cb_connect_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }

  stack++;

  // call back function when a connection fails
  if ((stack<=top) && (lua_isfunction(L, stack))){
    lua_pushvalue(L, stack);  // copy argument (func) to the top of stack
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_connect_fail_ref);
    mud->cb_connect_fail_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }

  lua_pushvalue(L, 1);  // copy userdata to the top of stack
  luaL_unref(L, LUA_REGISTRYINDEX, mud->self_ref);
  mud->self_ref = luaL_ref(L, LUA_REGISTRYINDEX);

  espconn_regist_connectcb(pesp_conn, mqtt_socket_connected);
  espconn_regist_reconcb(pesp_conn, mqtt_socket_reconnected);

  os_timer_disarm(&mud->mqttTimer);
  os_timer_setfn(&mud->mqttTimer, (os_timer_func_t *)mqtt_socket_timer, mud);
  SWTIMER_REG_CB(mqtt_socket_timer, SWTIMER_RESUME);
    //I assume that mqtt_socket_timer connects to the mqtt server, but I'm not really sure what impact light_sleep will have on it.
    //My guess: If in doubt, resume the timer
  // timer started in socket_connect()

  if((ipaddr.addr == IPADDR_NONE) && (memcmp(domain,"255.255.255.255",16) != 0))
  {
    ip_addr_t host_ip;
    switch (dns_gethostbyname(domain, &host_ip, socket_dns_foundcb, mud))
    {
      case ERR_OK:
        socket_dns_found(domain, &host_ip, mud);  // ip is returned in host_ip.
        break;
      case ERR_INPROGRESS:
        break;
      default:
        // Something has gone wrong; bail out?
        mqtt_connack_fail(mud, MQTT_CONN_FAIL_DNS);
    }
  }
  else
  {
    mqtt_socket_do_connect(mud);
  }

  NODE_DBG("leave mqtt_socket_connect.\n");

  return 0;
}

// Lua: mqtt:close()
// client disconnect and unref itself
static int mqtt_socket_close( lua_State* L )
{
  NODE_DBG("enter mqtt_socket_close.\n");
  int i = 0;
  lmqtt_userdata *mud = NULL;

  mud = (lmqtt_userdata *)luaL_checkudata(L, 1, "mqtt.socket");
  luaL_argcheck(L, mud, 1, "mqtt.socket expected");
  if (mud == NULL) {
    lua_pushboolean(L, 0);
    return 1;
  }

  sint8 espconn_status = ESPCONN_CONN;
  if (mud->connected) {
    uint8_t temp_buffer[MQTT_BUF_SIZE];
    mqtt_message_buffer_t msgb;
    mqtt_msg_init(&msgb, temp_buffer, MQTT_BUF_SIZE);

    // Send disconnect message
    mqtt_message_t* temp_msg = mqtt_msg_disconnect(&msgb);
    NODE_DBG("Send MQTT disconnect infomation, data len: %d, d[0]=%d \r\n", temp_msg->length,  temp_msg->data[0]);

    /* XXX This fails to actually send the disconnect message before hanging up */
#ifdef CLIENT_SSL_ENABLE
    if(mud->secure) {
      espconn_status = espconn_secure_send(&mud->pesp_conn, temp_msg->data, temp_msg->length);
      if(mud->pesp_conn.proto.tcp->remote_port || mud->pesp_conn.proto.tcp->local_port)
        espconn_status |= espconn_secure_disconnect(&mud->pesp_conn);
    } else
#endif
    {
      espconn_status = espconn_send(&mud->pesp_conn, temp_msg->data, temp_msg->length);
      if(mud->pesp_conn.proto.tcp->remote_port || mud->pesp_conn.proto.tcp->local_port)
        espconn_status |= espconn_disconnect(&mud->pesp_conn);
    }
  }
  mud->connected = false;

  while (mud->mqtt_state.pending_msg_q) {
    msg_destroy(msg_dequeue(&(mud->mqtt_state.pending_msg_q)));
  }

  NODE_DBG("leave mqtt_socket_close.\n");

  if (espconn_status == ESPCONN_OK) {
    lua_pushboolean(L, 1);
  } else {
    lua_pushboolean(L, 0);
  }
  return 1;
}

// Lua: mqtt:on( "method", function() )
static int mqtt_socket_on( lua_State* L )
{
  NODE_DBG("enter mqtt_socket_on.\n");
  lmqtt_userdata *mud;
  size_t sl;

  mud = (lmqtt_userdata *)luaL_checkudata(L, 1, "mqtt.socket");
  luaL_argcheck(L, mud, 1, "mqtt.socket expected");
  if(mud==NULL){
    NODE_DBG("userdata is nil.\n");
    return 0;
  }

  const char *method = luaL_checklstring( L, 2, &sl );
  if (method == NULL)
    return luaL_error( L, "wrong arg type" );

  luaL_checktype(L, 3, LUA_TFUNCTION);
  lua_pushvalue(L, 3);  // copy argument (func) to the top of stack

  if( sl == 7 && strcmp(method, "connect") == 0){
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_connect_ref);
    mud->cb_connect_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }else if( sl == 8 && strcmp(method, "connfail") == 0){
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_connect_fail_ref);
    mud->cb_connect_fail_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }else if( sl == 7 && strcmp(method, "offline") == 0){
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_disconnect_ref);
    mud->cb_disconnect_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }else if( sl == 7 && strcmp(method, "message") == 0){
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_message_ref);
    mud->cb_message_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }else if( sl == 8 && strcmp(method, "overflow") == 0){
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_overflow_ref);
    mud->cb_overflow_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }else if( sl == 6 && strcmp(method, "puback") == 0){
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_puback_ref);
    mud->cb_puback_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }else if( sl == 6 && strcmp(method, "suback") == 0){
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_suback_ref);
    mud->cb_suback_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }else if( sl == 8 && strcmp(method, "unsuback") == 0){
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_unsuback_ref);
    mud->cb_unsuback_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }else{
    lua_pop(L, 1);
    return luaL_error( L, "method not supported" );
  }
  NODE_DBG("leave mqtt_socket_on.\n");
  return 0;
}

// Lua: bool = mqtt:unsubscribe(topic, function())
static int mqtt_socket_unsubscribe( lua_State* L ) {
  NODE_DBG("enter mqtt_socket_unsubscribe.\n");

  uint8_t stack = 1;
  uint16_t msg_id;
  const char *topic;
  size_t il;
  lmqtt_userdata *mud;

  mud = (lmqtt_userdata *) luaL_checkudata( L, stack, "mqtt.socket" );
  luaL_argcheck( L, mud, stack, "mqtt.socket expected" );
  stack++;

  if(mud==NULL){
    NODE_DBG("userdata is nil.\n");
    lua_pushboolean(L, 0);
    return 1;
  }

  if(!mud->connected){
    luaL_error( L, "not connected" );
    lua_pushboolean(L, 0);
    return 1;
  }

  uint8_t temp_buffer[MQTT_BUF_SIZE];
  mqtt_message_buffer_t msgb;
  mqtt_msg_init(&msgb, temp_buffer, MQTT_BUF_SIZE);
  mqtt_message_t *temp_msg = NULL;

  if( lua_istable( L, stack ) ) {
    NODE_DBG("unsubscribe table\n");
    lua_pushnil( L ); /* first key */

    int topic_count = 0;
    uint8_t overflow = 0;

    while( lua_next( L, stack ) != 0 ) {
      topic = luaL_checkstring( L, -2 );

      if (topic_count == 0) {
        msg_id = mqtt_next_message_id(mud);
        temp_msg = mqtt_msg_unsubscribe_init( &msgb, msg_id );
      }
      temp_msg = mqtt_msg_unsubscribe_topic( &msgb, topic );
      topic_count++;

      NODE_DBG("topic: %s - length: %d\n", topic, temp_msg->length);

      if (temp_msg->length == 0) {
        lua_pop(L, 1);
        overflow = 1;
        break;  // too long message for the outbuffer.
      }

      lua_pop( L, 1 );
    }

    if (topic_count == 0){
      return luaL_error( L, "no topics found" );
    }
    if (overflow != 0){
      return luaL_error( L, "buffer overflow, can't enqueue all unsubscriptions" );
    }

    temp_msg = mqtt_msg_unsubscribe_fini( &msgb );
    if (temp_msg->length == 0) {
      return luaL_error( L, "buffer overflow, can't enqueue all unsubscriptions" );
    }

    stack++;
  } else {
    NODE_DBG("unsubscribe string\n");
    topic = luaL_checklstring( L, stack, &il );
    stack++;
    if( topic == NULL ){
      return luaL_error( L, "need topic name" );
    }
    msg_id = mqtt_next_message_id(mud);
    temp_msg = mqtt_msg_unsubscribe( &msgb, topic, msg_id );
  }

  if (lua_isfunction(L, stack)) {    // TODO: this will overwrite the previous one.
    lua_pushvalue( L, stack );          // copy argument (func) to the top of stack
    luaL_unref( L, LUA_REGISTRYINDEX, mud->cb_unsuback_ref );
    mud->cb_unsuback_ref = luaL_ref( L, LUA_REGISTRYINDEX );
  }

  msg_queue_t *node = msg_enqueue( &(mud->mqtt_state.pending_msg_q), temp_msg,
                                   msg_id, MQTT_MSG_TYPE_UNSUBSCRIBE, (int)mqtt_get_qos(temp_msg->data) );

  NODE_DBG("topic: %s - id: %d - qos: %d, length: %d\n", topic, node->msg_id, node->publish_qos, node->msg.length);
  NODE_DBG("msg_size: %d, event_timeout: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)), mud->event_timeout);

  sint8 espconn_status = ESPCONN_IF;

  espconn_status = mqtt_send_if_possible(mud);

  if(!node || espconn_status != ESPCONN_OK){
    lua_pushboolean(L, 0);
  } else {
    lua_pushboolean(L, 1);  // enqueued succeed.
  }
  NODE_DBG("unsubscribe, queue size: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)));
  NODE_DBG("leave mqtt_socket_unsubscribe.\n");
  return 1;
}

// Lua: bool = mqtt:subscribe(topic, qos, function())
static int mqtt_socket_subscribe( lua_State* L ) {
  NODE_DBG("enter mqtt_socket_subscribe.\n");

  uint8_t stack = 1, qos = 0;
  uint16_t msg_id;
  const char *topic;
  size_t il;
  lmqtt_userdata *mud;

  mud = (lmqtt_userdata *) luaL_checkudata( L, stack, "mqtt.socket" );
  luaL_argcheck( L, mud, stack, "mqtt.socket expected" );
  stack++;

  if(mud==NULL){
    NODE_DBG("userdata is nil.\n");
    lua_pushboolean(L, 0);
    return 1;
  }

  if(!mud->connected){
    luaL_error( L, "not connected" );
    lua_pushboolean(L, 0);
    return 1;
  }

  uint8_t temp_buffer[MQTT_BUF_SIZE];
  mqtt_message_buffer_t msgb;
  mqtt_msg_init(&msgb, temp_buffer, MQTT_BUF_SIZE);
  mqtt_message_t *temp_msg = NULL;

  if( lua_istable( L, stack ) ) {
    NODE_DBG("subscribe table\n");
    lua_pushnil( L ); /* first key */

    int topic_count = 0;
    uint8_t overflow = 0;

    while( lua_next( L, stack ) != 0 ) {
      topic = luaL_checkstring( L, -2 );
      qos = luaL_checkinteger( L, -1 );

      if (topic_count == 0) {
        msg_id = mqtt_next_message_id(mud);
        temp_msg = mqtt_msg_subscribe_init( &msgb, msg_id );
      }
      temp_msg = mqtt_msg_subscribe_topic( &msgb, topic, qos );
      topic_count++;

      NODE_DBG("topic: %s - qos: %d, length: %d\n", topic, qos, temp_msg->length);

      if (temp_msg->length == 0) {
        lua_pop(L, 1);
        overflow = 1;
        break;  // too long message for the outbuffer.
      }

      lua_pop( L, 1 );
    }

    if (topic_count == 0){
      return luaL_error( L, "no topics found" );
    }
    if (overflow != 0){
      return luaL_error( L, "buffer overflow, can't enqueue all subscriptions" );
    }

    temp_msg = mqtt_msg_subscribe_fini( &msgb );
    if (temp_msg->length == 0) {
      return luaL_error( L, "buffer overflow, can't enqueue all subscriptions" );
    }

    stack++;
  } else {
    NODE_DBG("subscribe string\n");
    topic = luaL_checklstring( L, stack, &il );
    stack++;
    if( topic == NULL ){
      return luaL_error( L, "need topic name" );
    }
    qos = luaL_checkinteger( L, stack );
    msg_id = mqtt_next_message_id(mud);
    temp_msg = mqtt_msg_subscribe( &msgb, topic, qos, msg_id );
    stack++;
  }

  if (lua_isfunction(L, stack)) {    // TODO: this will overwrite the previous one.
    lua_pushvalue( L, stack );  // copy argument (func) to the top of stack
    luaL_unref( L, LUA_REGISTRYINDEX, mud->cb_suback_ref );
    mud->cb_suback_ref = luaL_ref( L, LUA_REGISTRYINDEX );
  }

  msg_queue_t *node = msg_enqueue( &(mud->mqtt_state.pending_msg_q), temp_msg,
                                   msg_id, MQTT_MSG_TYPE_SUBSCRIBE, (int)mqtt_get_qos(temp_msg->data) );

  NODE_DBG("topic: %s - id: %d - qos: %d, length: %d\n", topic, node->msg_id, node->publish_qos, node->msg.length);
  NODE_DBG("msg_size: %d, event_timeout: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)), mud->event_timeout);

  sint8 espconn_status = ESPCONN_IF;

  espconn_status = mqtt_send_if_possible(mud);

  if(!node || espconn_status != ESPCONN_OK){
    lua_pushboolean(L, 0);
  } else {
    lua_pushboolean(L, 1);  // enqueued succeed.
  }
  NODE_DBG("subscribe, queue size: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)));
  NODE_DBG("leave mqtt_socket_subscribe.\n");
  return 1;
}

// Lua: bool = mqtt:publish( topic, payload, qos, retain, function() )
static int mqtt_socket_publish( lua_State* L )
{
  NODE_DBG("enter mqtt_socket_publish.\n");
  lmqtt_userdata *mud;
  size_t l;
  uint8_t stack = 1;
  uint16_t msg_id = 0;

  mud = (lmqtt_userdata *)luaL_checkudata(L, stack, "mqtt.socket");
  luaL_argcheck(L, mud, stack, "mqtt.socket expected");
  stack++;
  if(mud==NULL){
    NODE_DBG("userdata is nil.\n");
    lua_pushboolean(L, 0);
    return 1;
  }

  if(!mud->connected){
    return luaL_error( L, "not connected" );
  }

  const char *topic = luaL_checklstring( L, stack, &l );
  stack ++;
  if (topic == NULL){
    return luaL_error( L, "need topic" );
  }

  const char *payload = luaL_checklstring( L, stack, &l );
  stack ++;
  uint8_t qos = luaL_checkinteger( L, stack);
  stack ++;
  uint8_t retain = luaL_checkinteger( L, stack);
  stack ++;

  if (qos != 0) {
    msg_id = mqtt_next_message_id(mud);
  }

  uint8_t temp_buffer[MQTT_BUF_SIZE];
  mqtt_message_buffer_t msgb;
  mqtt_msg_init(&msgb, temp_buffer, MQTT_BUF_SIZE);
  mqtt_message_t *temp_msg = mqtt_msg_publish(&msgb,
                       topic, payload, l,
                       qos, retain,
                       msg_id);

  if (lua_isfunction(L, stack)){
    lua_pushvalue(L, stack);  // copy argument (func) to the top of stack
    luaL_unref(L, LUA_REGISTRYINDEX, mud->cb_puback_ref);
    mud->cb_puback_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }

  msg_queue_t *node = msg_enqueue(&(mud->mqtt_state.pending_msg_q), temp_msg,
                      msg_id, MQTT_MSG_TYPE_PUBLISH, (int)qos );

  sint8 espconn_status = ESPCONN_OK;

  espconn_status = mqtt_send_if_possible(mud);

  if(!node || espconn_status != ESPCONN_OK){
    lua_pushboolean(L, 0);
  } else {
    lua_pushboolean(L, 1);  // enqueued succeed.
  }

  NODE_DBG("publish, queue size: %d\n", msg_size(&(mud->mqtt_state.pending_msg_q)));
  NODE_DBG("leave mqtt_socket_publish.\n");
  return 1;
}

// Lua: mqtt:lwt( topic, message, qos, retain, function(client) )
static int mqtt_socket_lwt( lua_State* L )
{
  NODE_DBG("enter mqtt_socket_lwt.\n");
  uint8_t stack = 1;
  size_t topicSize, msgSize;
  NODE_DBG("mqtt_socket_lwt.\n");
  lmqtt_userdata *mud = NULL;
  const char *lwtTopic, *lwtMsg;
  uint8_t lwtQoS, lwtRetain;

  mud = (lmqtt_userdata *)luaL_checkudata( L, stack, "mqtt.socket" );
  luaL_argcheck( L, mud, stack, "mqtt.socket expected" );

  if(mud == NULL)
    return 0;

  stack++;
  lwtTopic = luaL_checklstring( L, stack, &topicSize );
  if (lwtTopic == NULL)
  {
    return luaL_error( L, "need lwt topic");
  }

  stack++;
  lwtMsg = luaL_checklstring( L, stack, &msgSize );
  if (lwtMsg == NULL)
  {
    return luaL_error( L, "need lwt message");
  }
  stack++;
  if(mud->connect_info.will_topic){    // free the previous one if there is any
    free(mud->connect_info.will_topic);
    mud->connect_info.will_topic = NULL;
  }
  if(mud->connect_info.will_message){
    free(mud->connect_info.will_message);
    mud->connect_info.will_message = NULL;
  }

  mud->connect_info.will_topic = (uint8_t*) calloc(1, topicSize + 1 );
  mud->connect_info.will_message = (uint8_t*) calloc(1, msgSize + 1 );
  if(!mud->connect_info.will_topic || !mud->connect_info.will_message){
    if(mud->connect_info.will_topic){
      free(mud->connect_info.will_topic);
      mud->connect_info.will_topic = NULL;
    }
    if(mud->connect_info.will_message){
      free(mud->connect_info.will_message);
      mud->connect_info.will_message = NULL;
    }
    return luaL_error( L, "not enough memory");
  }
  memcpy(mud->connect_info.will_topic, lwtTopic, topicSize);
  mud->connect_info.will_topic[topicSize] = 0;
  memcpy(mud->connect_info.will_message, lwtMsg, msgSize);
  mud->connect_info.will_message[msgSize] = 0;

  if ( lua_isnumber(L, stack) )
  {
    mud->connect_info.will_qos = lua_tointeger(L, stack);
    stack++;
  }
  if ( lua_isnumber(L, stack) )
  {
    mud->connect_info.will_retain = lua_tointeger(L, stack);
    stack++;
  }

  NODE_DBG("mqtt_socket_lwt: topic: %s, message: %s, qos: %d, retain: %d\n",
      mud->connect_info.will_topic,
      mud->connect_info.will_message,
      mud->connect_info.will_qos,
      mud->connect_info.will_retain);
  NODE_DBG("leave mqtt_socket_lwt.\n");
  return 0;
}

// Module function map

LROT_BEGIN(mqtt_socket, NULL, LROT_MASK_GC_INDEX)
  LROT_FUNCENTRY( __gc, mqtt_delete )
  LROT_TABENTRY(  __index, mqtt_socket )
  LROT_FUNCENTRY( connect, mqtt_socket_connect )
  LROT_FUNCENTRY( close, mqtt_socket_close )
  LROT_FUNCENTRY( publish, mqtt_socket_publish )
  LROT_FUNCENTRY( subscribe, mqtt_socket_subscribe )
  LROT_FUNCENTRY( unsubscribe, mqtt_socket_unsubscribe )
  LROT_FUNCENTRY( lwt, mqtt_socket_lwt )
  LROT_FUNCENTRY( on, mqtt_socket_on )
LROT_END(mqtt_socket, NULL, LROT_MASK_GC_INDEX)



LROT_BEGIN(mqtt, NULL, 0)
  LROT_FUNCENTRY( Client, mqtt_socket_client )
  LROT_NUMENTRY( CONN_FAIL_SERVER_NOT_FOUND, MQTT_CONN_FAIL_SERVER_NOT_FOUND )
  LROT_NUMENTRY( CONN_FAIL_NOT_A_CONNACK_MSG, MQTT_CONN_FAIL_NOT_A_CONNACK_MSG )
  LROT_NUMENTRY( CONN_FAIL_DNS, MQTT_CONN_FAIL_DNS )
  LROT_NUMENTRY( CONN_FAIL_TIMEOUT_RECEIVING, MQTT_CONN_FAIL_TIMEOUT_RECEIVING )
  LROT_NUMENTRY( CONN_FAIL_TIMEOUT_SENDING, MQTT_CONN_FAIL_TIMEOUT_SENDING )
  LROT_NUMENTRY( CONNACK_ACCEPTED, MQTT_CONNACK_ACCEPTED )
  LROT_NUMENTRY( CONNACK_REFUSED_PROTOCOL_VER, MQTT_CONNACK_REFUSED_PROTOCOL_VER )
  LROT_NUMENTRY( CONNACK_REFUSED_ID_REJECTED, MQTT_CONNACK_REFUSED_ID_REJECTED )
  LROT_NUMENTRY( CONNACK_REFUSED_SERVER_UNAVAILABLE, MQTT_CONNACK_REFUSED_SERVER_UNAVAILABLE )
  LROT_NUMENTRY( CONNACK_REFUSED_BAD_USER_OR_PASS, MQTT_CONNACK_REFUSED_BAD_USER_OR_PASS )
  LROT_NUMENTRY( CONNACK_REFUSED_NOT_AUTHORIZED, MQTT_CONNACK_REFUSED_NOT_AUTHORIZED )
LROT_END(mqtt, NULL, 0)


int luaopen_mqtt( lua_State *L )
{
  luaL_rometatable(L, "mqtt.socket", LROT_TABLEREF(mqtt_socket));
  return 0;
}

NODEMCU_MODULE(MQTT, "mqtt", mqtt, luaopen_mqtt);
