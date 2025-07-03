# Multithreading Architecture Reference

*A brief verbal explaination of how the multithreading in sofren works*

- No mutexes are used anywhere, as each thread works on a separate tile, and tiles are non overlapping rectangular regions of the screen, threads can write to the final buffers simultaneously without interfering with one another.

## 1: Initialization and Thread Creation

Everything is initialized from `sfr_init`, using memory from the provided buffer (`sfrBuffers`)

- Four semaphores, `geometryStartSem` (main thread signals this to tell workers to start geometry phase), `geometryDoneSem` (workers signal this upon completing their geometry work), `rasterStartSem` (main thread singlas this to tell workers to start raster phase), and `rasterDoneSem` (workers signal this upon completing their rasterization work), are initialized.
- `SFR_THREAD_COUNT` threads are created, with each executing `sfr__worker_thread_func`. On Windows, `_beginthreadex` is used, otherwise `pthread_create` is used.

## 2: The Worker Thread Main Loop

Once created, each thread enters the infinite loop inside `sfr__worker_thread_func` where they wait to be assigned work.

### 2.1: The Geometry Phase

Performs all the per triangle calculations independent of the final output (i.e. no writing to pixel or depth buffers), such as vertex transformation, lighting, and clipping.

1) **Job Creation (`sfr_mesh`):**
- When you call `sfr_mesh` with a large model, the work is broken up.
- The mesh's triangle list is divided into chunks, each `SFR_GEOMETRY_JOB_SIZE` in size.
- For each chunk, a `SfrMeshChunkJob` is created. This job contains a pointer to the mesh, the transformation matrices, color, texture, and the start index and count of triangles it's responsible for.

2) **Job Queuing (`sfr_mesh`):**
- Once a `SfrMeshChunkJob` is created, its index is added to the `geometryWorkQueue`.
- The `geometryWorkQueueCount` atomic is incremented to make the job visible to worker threads.

3) **Dispatch and Execution (`sfr_flush_and_wait` and `sfr__worker_thread_func`):**
- The main thread calling `sfr_flush_and_wait` posts to `geometryStartSem`, releasing all worker threads from their wait state.
- Each worker thread enters a loop to "steal" work from the `geometryWorkQueue`. It does this by atomically incrementing the `geometryWorkQueueHead` counter to get a unique job index.
- The thread then processes every triangle within its assigned `SfrMeshChunkJob` by calling `sfr__process_and_bin_triangle`.

4) **Triangle Binning (`sfr__process_and_bin_triangle` and `sfr__bin_triangle`):**
- The triangle's screen space bounding box is calculated to determine which screen tiles it overlaps (the screen is divided into a grid of tiles, each `SFR_TILE_WIDTH` by `SFR_TILE_HEIGHT` pixels).
- A pointer to the `SfrTriangleBin` is added to the bin list of every tile it touches. The `tile->binCount` is updated atomically.
- The first time a triangle is added to a specific tile, that tile's index is added to the `rasterWorkQueue`, marking it as needing rasterization in the next phase.

5) **Synchronization (`sfr_flush_and_wait`):**
- After a worker thread can't find more geometry jobs in the queue, it signals the `geometryDoneSem` and proceeds to wait on the `rasterStartSem` for the next phase.
- The main thread, inside `sfr_flush_and_wait`, waits until it has received a signal on `geometryDoneSem` from every worker thread, ensuring the entire geometry phase is complete before continuing.

### 2.2: The Rasterization Phase

Takes the binned triangles from the geometry phase and converts them to screen pixels.

1) **Job Creation:**
- The work for this phase was already created during the geometry phase.
- The `rasterWorkQueue` is now populated with the indices of all tiles that have at least one triangle to draw.

2) **Dispatch and Execution:**
- Still inside `sfr_flush_and_wait`, the main thread posts to the `rasterStartSem`, waking all worker threads to begin rasterization.
- Each worker atomically increments the `rasterWorkQueueHead` to steal a tile index from the `rasterWorkQueue`.
- The thread gets a pointer to the `SfrTile` and iterates through its list of `SfrTriangleBin` pointers. For each bin, it calls `sfr_rasterize_bin` to perform the actual rasterizing.

3) **Synchronization:**
- When a worker finishes its loop and the raster queue is empty, it signals the `rasterDoneSem` and loops back to wait for the next geometry phase.
- The main thread waits on `rasterDoneSem` until all workers have signaled completion, which marks the end of a fully rendered frame. It then resets all the job queues and allocators for the next frame.
