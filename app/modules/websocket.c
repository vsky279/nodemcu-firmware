// Module for websockets

// Example usage:
// ws = websocket.createClient()
// ws:on("connection", function() ws:send('hi') end)
// ws:on("receive", function(_, data, opcode) print(data) end)
// ws:on("close", function(_, reasonCode) print('ws closed', reasonCode) end)
// ws:connect('ws://echo.websocket.org')

#include "lmem.h"
#include "lauxlib.h"
#include "platform.h"
#include "module.h"

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "websocket/websocketclient.h"

#define METATABLE_WSCLIENT "websocket.client"

typedef struct ws_data {
  int self_ref;
  int onConnection;
  int onReceive;
  int onClose;
} ws_data;

static void websocketclient_onConnectionCallback(ws_info *ws) {
  NODE_DBG("websocketclient_onConnectionCallback\n");

  lua_State *L = lua_getstate();

  if (ws == NULL || ws->reservedData == NULL) {
    luaL_error(L, "Client websocket is nil.\n");
    return;
  }
  ws_data *data = (ws_data *) ws->reservedData;

  if (data->onConnection != LUA_NOREF) {
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->onConnection); // load the callback function
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->self_ref);  // pass itself, #1 callback argument
    luaL_pcallx(L, 1, 0);
  }
}

static void websocketclient_onReceiveCallback(ws_info *ws, int len, char *message, int opCode) {
  NODE_DBG("websocketclient_onReceiveCallback\n");

  lua_State *L = lua_getstate();

  if (ws == NULL || ws->reservedData == NULL) {
    luaL_error(L, "Client websocket is nil.\n");
    return;
  }
  ws_data *data = (ws_data *) ws->reservedData;

  if (data->onReceive != LUA_NOREF) {
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->onReceive); // load the callback function
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->self_ref);  // pass itself, #1 callback argument
    lua_pushlstring(L, message, len); // #2 callback argument
    lua_pushinteger(L, opCode); // #3 callback argument
    luaL_pcallx(L, 3, 0);
  }
}

static void websocketclient_onCloseCallback(ws_info *ws, int errorCode) {
  NODE_DBG("websocketclient_onCloseCallback\n");

  lua_State *L = lua_getstate();

  if (ws == NULL || ws->reservedData == NULL) {
    luaL_error(L, "Client websocket is nil.\n");
    return;
  }
  ws_data *data = (ws_data *) ws->reservedData;

  if (data->onClose != LUA_NOREF) {
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->onClose); // load the callback function
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->self_ref);  // pass itself, #1 callback argument
    lua_pushinteger(L, errorCode); // pass the error code, #2 callback argument
    luaL_pcallx(L, 2, 0);
  }

  // free self-reference to allow gc (no futher callback will be called until next ws:connect())
  lua_gc(L, LUA_GCSTOP, 0); // required to avoid freeing ws_data
  luaL_unref(L, LUA_REGISTRYINDEX, data->self_ref);
  data->self_ref = LUA_NOREF;
  lua_gc(L, LUA_GCRESTART, 0);
}

static int websocket_createClient(lua_State *L) {
  NODE_DBG("websocket_createClient\n");

  // create user data
  ws_data *data = (ws_data *) luaM_malloc(L, sizeof(ws_data));
  data->onConnection = LUA_NOREF;
  data->onReceive = LUA_NOREF;
  data->onClose = LUA_NOREF;
  data->self_ref = LUA_NOREF; // only set when ws:connect is called

  ws_info *ws = (ws_info *) lua_newuserdata(L, sizeof(ws_info));
  ws->connectionState = 0;
  ws->extraHeaders = NULL;
  ws->onConnection = &websocketclient_onConnectionCallback;
  ws->onReceive = &websocketclient_onReceiveCallback;
  ws->onFailure = &websocketclient_onCloseCallback;
  ws->reservedData = data;

  // set its metatable
  luaL_getmetatable(L, METATABLE_WSCLIENT);
  lua_setmetatable(L, -2);

  return 1;
}

static int websocketclient_on(lua_State *L) {
  NODE_DBG("websocketclient_on\n");

  ws_info *ws = (ws_info *) luaL_checkudata(L, 1, METATABLE_WSCLIENT);

  ws_data *data = (ws_data *) ws->reservedData;

  int handle = luaL_checkoption(L, 2, NULL, (const char * const[]){ "connection", "receive", "close", NULL });
  luaL_argcheck(L, lua_isnil(L,3) || lua_isfunction(L, 3), 3, "function or nil");

  switch (handle) {
    case 0:
      NODE_DBG("connection\n");

      luaL_unref(L, LUA_REGISTRYINDEX, data->onConnection);
      data->onConnection = LUA_NOREF;

      if (!lua_isnil(L,3)) {
        lua_pushvalue(L, 3);  // copy argument (func) to the top of stack
        data->onConnection = luaL_ref(L, LUA_REGISTRYINDEX);
      }
      break;
    case 1:
      NODE_DBG("receive\n");

      luaL_unref(L, LUA_REGISTRYINDEX, data->onReceive);
      data->onReceive = LUA_NOREF;

      if (!lua_isnil(L,3)) {
        lua_pushvalue(L, 3);  // copy argument (func) to the top of stack
        data->onReceive = luaL_ref(L, LUA_REGISTRYINDEX);
      }
      break;
    case 2:
      NODE_DBG("close\n");

      luaL_unref(L, LUA_REGISTRYINDEX, data->onClose);
      data->onClose = LUA_NOREF;

      if (!lua_isnil(L,3)) {
        lua_pushvalue(L, 3);  // copy argument (func) to the top of stack
        data->onClose = luaL_ref(L, LUA_REGISTRYINDEX);
      }
      break;
  }

  return 0;
}

static int websocketclient_connect(lua_State *L) {
  NODE_DBG("websocketclient_connect is called.\n");

  ws_info *ws = (ws_info *) luaL_checkudata(L, 1, METATABLE_WSCLIENT);

  ws_data *data = (ws_data *) ws->reservedData;

  if (ws->connectionState != 0 && ws->connectionState != 4) {
    return luaL_error(L, "Websocket already connecting or connected.\n");
  }
  ws->connectionState = 0;

  lua_pushvalue(L, 1);  // copy userdata to the top of stack to allow ref
  data->self_ref = luaL_ref(L, LUA_REGISTRYINDEX);

  const char *url = luaL_checkstring(L, 2);
  ws_connect(ws, url);

  return 0;
}

static header_t *realloc_headers(header_t *headers, int new_size) {
  if(headers) {
    for(header_t *header = headers; header->key; header++) {
      free(header->value);
      free(header->key);
    }
    free(headers);
  }
  if(!new_size)
    return NULL;
  return (header_t *)malloc(sizeof(header_t) * (new_size + 1));
}

static int websocketclient_config(lua_State *L) {
  NODE_DBG("websocketclient_config is called.\n");

  ws_info *ws = (ws_info *) luaL_checkudata(L, 1, METATABLE_WSCLIENT);

  ws_data *data = (ws_data *) ws->reservedData;

  luaL_checktype(L, 2, LUA_TTABLE);
  lua_getfield(L, 2, "headers");
  if(lua_istable(L, -1)) {

    lua_pushnil(L);
    int size = 0;
    while(lua_next(L, -2)) {
      size++;
      lua_pop(L, 1);
    }

    ws->extraHeaders = realloc_headers(ws->extraHeaders, size);
    if(ws->extraHeaders) {
      header_t *header = ws->extraHeaders;

      lua_pushnil(L);
      while(lua_next(L, -2)) {
        header->key = strdup(lua_tostring(L, -2));
        header->value = strdup(lua_tostring(L, -1));
        header++;
        lua_pop(L, 1);
      }

      header->key = header->value = NULL;
    }
  }
  lua_pop(L, 1); // pop headers

  return 0;
}

static int websocketclient_send(lua_State *L) {
  NODE_DBG("websocketclient_send is called.\n");

  ws_info *ws = (ws_info *) luaL_checkudata(L, 1, METATABLE_WSCLIENT);

  ws_data *data = (ws_data *) ws->reservedData;

  if (ws->connectionState != 3) {
    // should this be an onFailure callback instead?
    return luaL_error(L, "Websocket isn't connected.\n");
  }

  int msgLength;
  const char *msg = luaL_checklstring(L, 2, &msgLength);

  int opCode = 1; // default: text message
  if (lua_gettop(L) == 3) {
    opCode = luaL_checkint(L, 3);
  }

  ws_send(ws, opCode, msg, (unsigned short) msgLength);
  return 0;
}

static int websocketclient_close(lua_State *L) {
  NODE_DBG("websocketclient_close.\n");
  ws_info *ws = (ws_info *) luaL_checkudata(L, 1, METATABLE_WSCLIENT);

  ws_close(ws);
  return 0;
}

static int websocketclient_gc(lua_State *L) {
  NODE_DBG("websocketclient_gc\n");

  ws_info *ws = (ws_info *) luaL_checkudata(L, 1, METATABLE_WSCLIENT);

  ws->extraHeaders = realloc_headers(ws->extraHeaders, 0);

  ws_data *data = (ws_data *) ws->reservedData;

  luaL_unref(L, LUA_REGISTRYINDEX, data->onConnection);
  luaL_unref(L, LUA_REGISTRYINDEX, data->onReceive);

  if (data->onClose != LUA_NOREF) {
    if (ws->connectionState != 4) { // only call if connection open
      lua_rawgeti(L, LUA_REGISTRYINDEX, data->onClose);

      lua_pushinteger(L, -100);
      luaL_pcallx(L, 1, 0);
    }
    luaL_unref(L, LUA_REGISTRYINDEX, data->onClose);
  }

  if (data->self_ref != LUA_NOREF) {
    lua_gc(L, LUA_GCSTOP, 0); // required to avoid freeing ws_data
    luaL_unref(L, LUA_REGISTRYINDEX, data->self_ref);
    data->self_ref = LUA_NOREF;
    lua_gc(L, LUA_GCRESTART, 0);
  }

  NODE_DBG("freeing lua data\n");
  luaM_free(L, data);
  NODE_DBG("done freeing lua data\n");

  return 0;
}

LROT_BEGIN(websocket, NULL, 0)
  LROT_FUNCENTRY( createClient, websocket_createClient )
LROT_END(websocket, NULL, 0)



LROT_BEGIN(websocketclient, NULL, LROT_MASK_GC_INDEX)
  LROT_FUNCENTRY( __gc, websocketclient_gc )
  LROT_TABENTRY(  __index, websocketclient )
  LROT_FUNCENTRY( on, websocketclient_on )
  LROT_FUNCENTRY( config, websocketclient_config )
  LROT_FUNCENTRY( connect, websocketclient_connect )
  LROT_FUNCENTRY( send, websocketclient_send )
  LROT_FUNCENTRY( close, websocketclient_close )
LROT_END(websocketclient, NULL, LROT_MASK_GC_INDEX)


int loadWebsocketModule(lua_State *L) {
  luaL_rometatable(L, METATABLE_WSCLIENT, LROT_TABLEREF( websocketclient));

  return 0;
}

NODEMCU_MODULE(WEBSOCKET, "websocket", websocket, loadWebsocketModule);
