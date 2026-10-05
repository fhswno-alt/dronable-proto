# Moondream2 scene ask (Prefer FAIL)

OWL-ViT object votes stay the held-out Prefer FAIL record in `docs/FIND_ROOM_HELD_OUT.md`. This path does not edit that object map. It asks one local model which room a still is. No gait, no vel, no arrival.

## Pin and licence

Hub model `vikhyatk/moondream2`. The pinned snapshot is `5d6c926f44e26b07957b0dd315bbedcb4c17a5fe`. Weight blob sha256 `70a7d94c0c8349eb58ed2d9e636ef2d0916960f321ecabeac6354b8ba3d7403f`.

The model card at that snapshot begins:

```
license: apache-2.0
```

The card also names revision `2025-06-21` (`9a7d4024050840e001defacec2b00727e89149e6`). Those are the same weights. On transformers 5.18.0 that tag fails to load:

`AttributeError: 'HfMoondream' object has no attribute 'all_tied_weights_keys'.`

The wired path loads the snapshot above, which does load.

## Prompt and parse

Prompt, frozen before the fresh score:

`Which room is this? Answer with one word: kitchen, bathroom, living, bedroom, entrance, or none.`

Parse: lowercase, treat "living room" as living, split on spaces and on `/` `,` `;`. One room word and no none-word returns that room. Only a none-word (`none`, `unknown`, `unsure`, `undecided`) returns `none`. Two room words, a room word plus a none-word, or no allowed word returns no room. The caller keeps that frame undecided.

`confidence` is the greedy probability of the first answer token. `raw` is the model string. A hedge still has no room.

## Selection versus fresh

Selection, already used to pick Moondream, is not this score. It was the unpinned same snapshot and a prompt that did not offer `none`. Those stills are FloorPlan1–5, 201–205, 301–305, 401–405, and the first five Places365 validation files of kitchen, bathroom, bedroom, living, entrance hall, and corridor. That run named 20/20 iTHOR rooms and 26/30 Places photos.

Fresh iTHOR, rendered once at 0.38 m and 104.82° and then scored: FloorPlan6–10, 206–210, 306–310, 406–410.

Fresh Places ids:

| Class | Files |
|--|--|
| Kitchen | 00001691, 00001940, 00002112, 00002295, 00002312 |
| Bathroom | 00002198, 00002239, 00002398, 00002575, 00002580 |
| Bedroom | 00001822, 00003614, 00003929, 00004018, 00004922 |
| Living | 00005291, 00005689, 00005899, 00005909, 00006199 |
| Entrance hall | 00001503, 00001730, 00002003, 00002170, 00002424 |
| Corridor | 00002272, 00002774, 00003168, 00004729, 00004785 |

## Final score

| Source | Scene | Named hit | Unknown | Wrong room |
|--|--|--|--|--|
| iTHOR fresh | Kitchen | 3/5 | 0/5 | 2/5 |
| iTHOR fresh | Living | 5/5 | 0/5 | 0/5 |
| iTHOR fresh | Bedroom | 2/5 | 0/5 | 3/5 |
| iTHOR fresh | Bathroom | 5/5 | 0/5 | 0/5 |
| Places fresh | Kitchen | 5/5 | 0/5 | 0/5 |
| Places fresh | Bathroom | 5/5 | 0/5 | 0/5 |
| Places fresh | Bedroom | 5/5 | 0/5 | 0/5 |
| Places fresh | Living | 5/5 | 0/5 | 0/5 |
| Places fresh | Entrance hall | 3/5 | 0/5 | 2/5 |
| Places fresh | Corridor | 5/5 | 0/5 | 0/5 |
| Kit_cam | Kitchen | 1/1 | 0/1 | 0/1 |
| Kit_cam | Bathroom | 1/1 | 0/1 | 0/1 |
| Kit_cam | Living | 1/1 | 0/1 | 0/1 |
| Kit_cam | Bedroom | 1/1 | 0/1 | 0/1 |
| Kit_cam | Empty plant | 0/1 | 1/1 | 0/1 |

Fresh iTHOR is **15/20**. The five wrong names are living or entrance on a kitchen, and living on a bedroom. Fresh Places is **28/30**. Two entrance-hall photos were named living. The empty plant answer was `None`, parsed as none. A corridor named entrance counts as a hit. Unknown on a corridor would have been allowed.

Kit_cam entrance is untested. There is no Hardware hallway in this tree, and `room_entrance.xml` was not scored as one.

## Latency

55 answers on this agent VM, 4x Intel Xeon. Median **5.16 s**, 95th percentile **6.28 s**. This is not Dave's M4. A which-room question every few steps fits that cost. Every frame does not.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`.
