---
id: 9
title: Resolve Backend Connection Errors
stage: misc
date: 2025-12-18
surface: agent
model: gemini
feature: 002-rag-backend
branch: 002-rag-backend
user: user
command: when i run my command i backend folder so tis error is coming in backend terminal...
labels: ["backend", "connection-error", "qdrant", "voyage-ai"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - rag-backend/README.md
 - rag-backend/src/core/embeddings.py
 - rag-backend/src/scripts/ingest.py
 - rag-backend/src/core/qdrant.py
tests:
 - none
---

## Prompt

when i run my command i backend folder so tis error is coming in backend terminal PS D:\hackaton-specify\humanoid-book\rag-backend> poetry run python -m src.scripts.ingest
>>
2025-12-18 14:34:09.682 | INFO     | __main__:ingest_content:29 - Starting content ingestion process.
Traceback (most recent call last):
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpx\_transports\default.py", line 101, in map_httpcore_exceptions
    yield
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpx\_transports\default.py", line 250, in handle_request
    resp = self._pool.handle_request(req)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpcore\_sync\connection_pool.py", line 256, in handle_request
    raise exc from None
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpcore\_sync\connection_pool.py", line 236, in handle_request
    response = connection.handle_request(
        pool_request.request
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpcore\_sync\connection.py", line 101, in handle_request
    raise exc
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpcore\_sync\connection.py", line 78, in handle_request
    stream = self._connect(request)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpcore\_sync\connection.py", line 124, in _connect
    stream = self._network_backend.connect_tcp(**kwargs)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpcore\_backends\sync.py", line 207, in connect_tcp
    with map_exceptions(exc_map):
         ~~~~~~~~~~~~~~^^^^^^^^^
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\contextlib.py", line 162, in __exit__
    self.gen.throw(value)
    ~~~~~~~~~~~~~~^^^^^^^
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpcore\_exceptions.py", line 14, in map_exceptions
    raise to_exc(exc) from exc
httpcore.ConnectError: [WinError 10061] No connection could be made because the target machine actively refused it

The above exception was the direct cause of the following exception:

Traceback (most recent call last):
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\http\api_client.py", line 134, in send_inner
    response = self._client.send(request)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpx\_client.py", line 914, in send
    response = self._send_handling_auth( ...<2 lines>... 
        history=[],
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpx\_client.py", line 942, in _send_handling_auth
    response = self._send_handling_redirects(
        request,
        follow_redirects=follow_redirects,
        history=history,
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpx\_client.py", line 979, in _send_handling_redirects
    response = self._send_single_request(request)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpx\_client.py", line 1014, in _send_single_request
    response = transport.handle_request(request)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpx\_transports\default.py", line 249, in handle_request
    with map_httpcore_exceptions():
         ~~~~~~~~~~~~~~~~~~~~~~~^^
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\contextlib.py", line 162, in __exit__
    self.gen.throw(value)
    ~~~~~~~~~~~~~~^^^^^^^
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\httpx\_transports\default.py", line 118, in map_httpcore_exceptions
    raise mapped_exc(message) from exc
httpx.ConnectError: [WinError 10061] No connection could be made because the target machine actively refused it

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "<frozen runpy>", line 198, in _run_module_as_main
  File "<frozen runpy>", line 88, in _run_code
  File "D:\hackaton-specify\humanoid-book\rag-backend\src\scripts\ingest.py", line 111, in <module>
    ingest_content()
    ~~~~~~~~~~~~~~^^ 
  File "D:\hackaton-specify\humanoid-book\rag-backend\src\scripts\ingest.py", line 34, in ingest_content
    if not qdrant_client.collection_exists(collection_name=COLLECTION_NAME):
           ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\qdrant_client.py", line 1547, in collection_exists
    return self._client.collection_exists(collection_name=collection_name, **kwargs)
           ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\qdrant_remote.py", line 1812, in collection_exists
    result: models.CollectionExistence | None = self.http.collections_api.collection_exists(
                                                ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^ 
        collection_name=collection_name
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ).result
    ^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\http\api\collections_api.py", line 281, in collection_exists
    return self._build_for_collection_exists( ...
           ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^ 
        collection_name=collection_name,
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    )
    ^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\http\api\collections_api.py", line 67, in _build_for_collection_exists
    return self.api_client.request(
           ~~~~~~~~~~~~~~~~~~~~~~~^ 
        type_=m.InlineResponse2006,
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ...<3 lines>... 
        path_params=path_params,
        ^^^^^^^^^^^^^^^^^^^^^^^^
    )
    ^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\http\api_client.py", line 95, in request
    return self.send(request, type_)
           ~~~~~~~~~^^^^^^^^^^^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\http\api_client.py", line 112, in send
    response = self.middleware(request, self.send_inner)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\http\api_client.py", line 250, in __call__
    return call_next(request)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\qdrant_client\http\api_client.py", line 136, in send_inner
    raise ResponseHandlingException(e)
qdrant_client.http.exceptions.ResponseHandlingException: [WinError 10061] No connection could be made because the target machine actively refused it
PS D:\hackaton-specify\humanoid-book\rag-backend> poetry run uvicorn src.main:app --reload --port 8000
INFO:     Will watch for changes in these directories: [\'D:\\hackaton-specify\\humanoid-book\\rag-backend\]
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [9412] using StatReload
D:\hackaton-specify\humanoid-book\rag-backend\src\services\rag_service.py:1: FutureWarning: 

All support for the `google.generativeai` package has ended. It will no longer be receiving
updates or bug fixes. Please switch to the `google.genai` package as soon as possible.
See README for more details:

https://github.com/google-gemini/deprecated-generative-ai-python/blob/main/README.md

  import google.generativeai as genai
FastAPI CORS configured with ALLOWED_ORIGINS: [\'http://localhost:3000\', \'http://127.0.0.1:3000\']
INFO:     Started server process [4556]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     127.0.0.1:65064 - "OPTIONS /api/v1/query HTTP/1.1" 200 OK
2025-12-18 14:15:28.776 | INFO     | src.services.rag_service:query_general:19 - Received general query: tell me about gazebo
WARNING:urllib3.connectionpool:Retrying (Retry(total=1, connect=None, read=None, redirect=None, status=None)) after connection broken by 'NewConnectionError("HTTPSConnection(host=\'api.voyageai.com\', port=443): Failed to establish a new connection: [WinError 10061] No connection could be made because the target machine actively refused it")': /v1/embeddings
WARNING:urllib3.connectionpool:Retrying (Retry(total=0, connect=None, read=None, redirect=None, status=None)) after connection broken by 'NewConnectionError("HTTPSConnection(host=\'api.voyageai.com\', port=443): Failed to establish a new connection: [WinError 10061] No connection could be made because the target machine actively refused it")': /v1/embeddings
2025-12-18 14:15:35.184 | ERROR    | src.core.exceptions:validation_exception_handler:26 - Validation/Generic Exception: Error communicating with VoyageAI: HTTPSConnectionPool(host=\'api.voyageai.com\', port=443): Max retries exceeded with url: /v1/embeddings (Caused by NewConnectionError("HTTPSConnection(host=\'api.voyageai.com\', port=443): Failed to establish a new connection: [WinError 10061] No connection could be made because the target machine actively refused it"))
INFO:     127.0.0.1:65064 - "POST /api/v1/query HTTP/1.1" 422 Unprocessable Content
ERROR:    Exception in ASGI application
Traceback (most recent call last):
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connection.py", line 204, in _new_conn
    sock = connection.create_connection(
        (self._dns_host, self.port),
    ...<2 lines>... 
        socket_options=self.socket_options,
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\util\connection.py", line 85, in create_connection
    raise err
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\util\connection.py", line 73, in create_connection
    sock.connect(sa)
    ~~~~~~~~~~~~^^^^
ConnectionRefusedError: [WinError 10061] No connection could be made because the target machine actively refused it

The above exception was the direct cause of the following exception:

Traceback (most recent call last):
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connectionpool.py", line 787, in urlopen
    response = self._make_request(
        conn,
    ...<10 lines>... 
        **response_kw,
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connectionpool.py", line 488, in _make_request
    raise new_e
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connectionpool.py", line 464, in _make_request
    self._validate_conn(conn)
    ~~~~~~~~~~~~~~~~~~~^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connectionpool.py", line 1093, in _validate_conn
    conn.connect()
    ~~~~~~~~~~~~^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connection.py", line 759, in connect
    self.sock = sock = self._new_conn()
                       ~~~~~~~~~~~~~~^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connection.py", line 219, in _new_conn
    raise NewConnectionError(
        self, f"Failed to establish a new connection: {e}"
    ) from e
urllib3.exceptions.NewConnectionError: HTTPSConnection(host=\'api.voyageai.com\', port=443): Failed to establish a new connection: [WinError 10061] No connection could be made because the target machine actively refused it

The above exception was the direct cause of the following exception:

Traceback (most recent call last):
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\requests\adapters.py", line 644, in send
    resp = conn.urlopen(
        method=request.method,
    ...<9 lines>... 
        chunked=chunked,
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connectionpool.py", line 871, in urlopen
    return self.urlopen(
           ~~~~~~~~~~~~^ 
        method,
        ^^^^^^^ 
    ...<13 lines>... 
        **response_kw,
    )
    ^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connectionpool.py", line 871, in urlopen
    return self.urlopen(
           ~~~~~~~~~~~~^ 
        method,
        ^^^^^^^ 
    ...<13 lines>... 
        **response_kw,
    )
    ^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\connectionpool.py", line 841, in urlopen
    retries = retries.increment(
        method, url, error=new_e, _pool=self, _stacktrace=sys.exc_info()[2]
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\urllib3\util\retry.py", line 519, in increment
    raise MaxRetryError(_pool, url, reason) from reason  # type: ignore[arg-type]
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
urllib3.exceptions.MaxRetryError: HTTPSConnectionPool(host=\'api.voyageai.com\', port=443): Max retries exceeded with url: /v1/embeddings (Caused by NewConnectionError("HTTPSConnection(host=\'api.voyageai.com\', port=443): Failed to establish a new connection: [WinError 10061] No connection could be made because the target machine actively refused it"))

The above exception was the direct cause of the following exception:

Traceback (most recent call last):
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\voyageai\api_resources\api_requestor.py", line 543, in request_raw
    result = _thread_context.session.request(
        method, ...<6 lines>... 
        proxies=_thread_context.session.proxies,
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\requests\sessions.py", line 589, in request
    resp = self.send(prep, **send_kwargs)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\requests\sessions.py", line 703, in send
    r = adapter.send(request, **kwargs)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\requests\adapters.py", line 677, in send
    raise ConnectionError(e, request=request)
requests.exceptions.ConnectionError: HTTPSConnectionPool(host=\'api.voyageai.com\', port=443): Max retries exceeded with url: /v1/embeddings (Caused by NewConnectionError("HTTPSConnection(host=\'api.voyageai.com\', port=443): Failed to establish a new connection: [WinError 10061] No connection could be made because the target machine actively refused it"))

The above exception was the direct cause of the following exception:

Traceback (most recent call last):
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\uvicorn\protocols\http\h11_impl.py", line 403, in run_asgi
    result = await app(  # type: ignore[func-returns-value]
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
        self.scope, self.receive, self.send
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\uvicorn\middleware\proxy_headers.py", line 60, in __call__
    return await self.app(scope, receive, send)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\fastapi\applications.py", line 1139, in __call__
    await super().__call__(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\applications.py", line 107, in __call__
    await self.middleware_stack(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\middleware\errors.py", line 186, in __call__
    raise exc
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\middleware\errors.py", line 164, in __call__
    await self.app(scope, receive, _send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\middleware\cors.py", line 93, in __call__
    await self.simple_response(scope, receive, send, request_headers=headers)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\middleware\cors.py", line 144, in simple_response
    await self.app(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\middleware\exceptions.py", line 63, in __call__
    await wrap_app_handling_exceptions(self.app, conn)(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\_exception_handler.py", line 53, in wrapped_app
    raise exc
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\_exception_handler.py", line 42, in wrapped_app
    await app(scope, receive, sender)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\fastapi\middleware\asyncexitstack.py", line 18, in __call__
    await self.app(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\routing.py", line 716, in __call__
    await self.middleware_stack(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\routing.py", line 736, in app
    await route.handle(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\routing.py", line 290, in handle
    await self.app(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\fastapi\routing.py", line 120, in app
    await wrap_app_handling_exceptions(app, request)(scope, receive, send)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\_exception_handler.py", line 53, in wrapped_app
    raise exc
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\_exception_handler.py", line 42, in wrapped_app
    await app(scope, receive, sender)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\fastapi\routing.py", line 106, in app
    response = await f(request)
               ^^^^^^^^^^^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\fastapi\routing.py", line 430, in app
    raw_response = await run_endpoint_function( ...
                   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
    ...<3 lines>... 
    )
    ^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\fastapi\routing.py", line 318, in run_endpoint_function
    return await run_in_threadpool(dependant.call, **values)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\starlette\concurrency.py", line 32, in run_in_threadpool
    return await anyio.to_thread.run_sync(func)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\anyio\to_thread.py", line 61, in run_sync
    return await get_async_backend().run_sync_in_worker_thread( ...
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
        func, args, abandon_on_cancel=abandon_on_cancel, limiter=limiter
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
    )
    ^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\anyio\_backends\_asyncio.py", line 2525, in run_sync_in_worker_thread
    return await future
           ^^^^^^^^^^^^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\anyio\_backends\_asyncio.py", line 986, in run
    result = context.run(func, *args)
  File "D:\hackaton-specify\humanoid-book\rag-backend\src\api\query.py", line 13, in query_general
    return rag_service.query_general(request.query)
           ~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^ 
  File "D:\hackaton-specify\humanoid-book\rag-backend\src\services\rag_service.py", line 20, in query_general
    query_embedding = get_embedding(query)
  File "D:\hackaton-specify\humanoid-book\rag-backend\src\core\embeddings.py", line 16, in get_embedding
    response = voyage_client.embed([text], model="voyage-2")
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\voyageai\client.py", line 36, in embed
    response = voyageai.Embedding.create(
        input=texts,
    ...<3 lines>... 
        **self._params,
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\voyageai\api_resources\embedding.py", line 29, in create
    response = super().create(*args, **kwargs)
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\voyageai\api_resources\abstract\engine_api_resource.py", line 115, in create
    response, _, api_key = requestor.request(
                           ~~~~~~~~~~~~~~~~~^ 
        "post",
        ^^^^^^^ 
    ...<5 lines>... 
        request_timeout=request_timeout,
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    )
    ^ 
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\voyageai\api_resources\api_requestor.py", line 289, in request
    result = self.request_raw(
        method.lower(), ...<6 lines>... 
        request_timeout=request_timeout,
    )
  File "C:\Users\cv\AppData\Local\pypoetry\Cache\virtualenvs\rag-backend-vF4WmWaX-py3.13\Lib\site-packages\voyageai\api_resources\api_requestor.py", line 556, in request_raw
    raise error.APIConnectionError(
        "Error communicating with VoyageAI: {}".format(e)
    ) from e
voyageai.error.APIConnectionError: Error communicating with VoyageAI: HTTPSConnectionPool(host=\'api.voyageai.com\', port=443): Max retries exceeded with url: /v1/embeddings (Caused by NewConnectionError("HTTPSConnection(host=\'api.voyageai.com\', port=443): Failed to establish a new connection: [WinError 10061] No connection could be made because the target machine actively refused it")) in browser the coes error is coming

## Response snapshot

The backend is failing to connect to Qdrant Cloud and the Voyage AI API. The errors indicate that the connection is being refused, which is likely due to incorrect environment variables (`QDRANT_URL`, `QDRANT_API_KEY`, `VOYAGE_API_KEY`) in the `.env` file or a network issue (firewall/proxy) blocking the connections. The user was advised to check these settings.

## Outcome

- ✅ Impact: Diagnosed the root cause of the backend connection errors and provided the user with clear, actionable steps to resolve them by checking their environment variables and network settings.
- 🧪 Tests: none
- 📁 Files: 
 - rag-backend/README.md
 - rag-backend/src/core/embeddings.py
 - rag-backend/src/scripts/ingest.py
 - rag-backend/src/core/qdrant.py
- 🔁 Next prompts: none
- 🧠 Reflection: The `README.md` was out of date regarding the embedding provider, causing initial confusion. Directly inspecting the code (`embeddings.py`) was crucial to identify the actual dependency (Voyage AI). When a script fails, check for alternative methods described in the project's meta-files (`GEMINI.md`).

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.ps1` script was not found at the specified path, preventing automatic PHR creation.
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Verify the existence of a script before attempting to execute it. Use `list_directory` to confirm the path.
