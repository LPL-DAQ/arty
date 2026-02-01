use tonic::{Code, Status};

pub trait MapTonicErr<T> {
    fn map_tonic_err(self, code: Code) -> Result<T, Status>;
}

impl<T> MapTonicErr<T> for Result<T, anyhow::Error> {
    fn map_tonic_err(self, code: Code) -> Result<T, Status> {
        self.map_err(|err| Status::new(code, format!("{err:#}")))
    }
}
