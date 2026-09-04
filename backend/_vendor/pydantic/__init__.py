"""
Vendored minimal pydantic shim.
Provides just enough of the pydantic v2 API for models.py to import and work.
This is a DEVELOPMENT-ONLY shim for testing on air-gapped machines.
Replace with real pydantic when internet is available.
"""

from dataclasses import dataclass, field as _dc_field, fields as _dc_fields
from typing import Any, Callable, List, Optional, get_type_hints
import json
import copy


def Field(default=..., *, ge=None, le=None, **kwargs):
    """Minimal pydantic.Field replacement."""
    if default is ...:
        return _dc_field(default_factory=lambda: None)
    return _dc_field(default=default)


def field_validator(*field_names, mode='before'):
    """Decorator shim for pydantic v2 field_validator."""
    def decorator(func):
        func._field_validator_names = field_names
        return func
    return decorator


class _ModelMeta(type):
    """Metaclass that converts class annotations into a dataclass-like init."""
    def __new__(mcs, name, bases, namespace):
        cls = super().__new__(mcs, name, bases, namespace)
        if name == 'BaseModel':
            return cls

        # Gather annotations
        annotations = {}
        for base in reversed(cls.__mro__):
            annotations.update(getattr(base, '__annotations__', {}))

        cls.__annotations__ = annotations
        return cls


class BaseModel(metaclass=_ModelMeta):
    """Minimal pydantic BaseModel replacement using plain __init__."""

    def __init__(self, **kwargs):
        hints = {}
        for klass in reversed(type(self).__mro__):
            hints.update(getattr(klass, '__annotations__', {}))

        # Run field validators
        validators = {}
        for attr_name in dir(type(self)):
            attr = getattr(type(self), attr_name, None)
            if callable(attr) and hasattr(attr, '_field_validator_names'):
                for fn in attr._field_validator_names:
                    validators[fn] = attr

        for field_name, field_type in hints.items():
            if field_name in kwargs:
                value = kwargs[field_name]
                # Run validator if exists
                if field_name in validators:
                    value = validators[field_name](value)
                setattr(self, field_name, value)
            elif hasattr(type(self), field_name):
                default = getattr(type(self), field_name)
                if isinstance(default, list):
                    setattr(self, field_name, list(default))
                elif isinstance(default, dict):
                    setattr(self, field_name, dict(default))
                else:
                    setattr(self, field_name, default)
            else:
                # Check if Optional
                origin = getattr(field_type, '__origin__', None)
                if origin is type(None) or (hasattr(field_type, '__args__') and type(None) in getattr(field_type, '__args__', ())):
                    setattr(self, field_name, None)
                else:
                    # Try to use type default
                    if field_type in (int, float):
                        setattr(self, field_name, field_type())
                    elif field_type is str:
                        setattr(self, field_name, "")
                    elif field_type is bool:
                        setattr(self, field_name, False)
                    elif field_type is list or (hasattr(field_type, '__origin__') and getattr(field_type, '__origin__', None) is list):
                        setattr(self, field_name, [])
                    else:
                        setattr(self, field_name, None)

    def model_dump(self) -> dict:
        result = {}
        hints = {}
        for klass in reversed(type(self).__mro__):
            hints.update(getattr(klass, '__annotations__', {}))
        for field_name in hints:
            val = getattr(self, field_name, None)
            if isinstance(val, BaseModel):
                result[field_name] = val.model_dump()
            elif isinstance(val, list):
                result[field_name] = [
                    item.model_dump() if isinstance(item, BaseModel) else item
                    for item in val
                ]
            else:
                result[field_name] = val
        return result

    def model_dump_json(self) -> str:
        return json.dumps(self.model_dump(), default=str)

    def model_copy(self, **kwargs):
        data = self.model_dump()
        data.update(kwargs)
        return type(self)(**data)

    def __repr__(self):
        hints = {}
        for klass in reversed(type(self).__mro__):
            hints.update(getattr(klass, '__annotations__', {}))
        fields = ", ".join(f"{k}={getattr(self, k, None)!r}" for k in hints)
        return f"{type(self).__name__}({fields})"


# Type alias shims
from typing import Literal
