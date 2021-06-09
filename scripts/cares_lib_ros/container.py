

def transpose_list_dicts(xs, dict_type=None):
  dict_type = dict_type or xs[0].__class__

  r = dict_type({})
  for d in xs:
    for k, v in d.items():
      inner = r.get(k) or []
      inner.append(v)
      r[k] = inner
  return r


def split_list(xs, n):
  return xs[n:], xs[:n]
